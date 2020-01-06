//
// Created by Kiss Levente on 2019. 12. 17..
//

#include "SimpleICP.h"

bool SimpleICP::iterate(
        Eigen::Matrix3f& R,
        Eigen::Vector3f& t,
        float& error,
        float eps) {

    //Number of nearest neighbours
    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    float oldError = 0;

    _cm = Eigen::Vector3f::Zero();
    _cd = Eigen::Vector3f::Zero();

    auto * pairs (new std::vector<struct PointPair>);

    for(auto i = 0; i < _D->size(); ++i)
    {
        if(_kdTree.nearestKSearch(_D->points[i],K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            pairs->push_back(PointPair(pointNKNSquaredDistance[0],pointIdxNKNSearch[0],i));

            _cm += _M->points[pointIdxNKNSearch[0]].getVector3fMap();
            _cd += _D->points[i].getVector3fMap();

            oldError += std::powf(_M->points[pointIdxNKNSearch[0]].x - _D->points[i].x, 2)
                        + std::powf(_M->points[pointIdxNKNSearch[0]].y - _D->points[i].y, 2) +
                        + std::powf(_M->points[pointIdxNKNSearch[0]].z - _D->points[i].z, 2);
        }
    }

    oldError /= _D->size();
    _cm /= pairs->size();
    _cd /= pairs->size();


    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f dataPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f modelPoint = Eigen::Vector3f::Zero();


    for(auto & pair : *pairs) {

        dataPoint = _D->points[pair.dataPointIndex].getVector3fMap();
        modelPoint = _M->points[pair.modelPointIndex].getVector3fMap();
        //Remove means

        dataPoint -= _cd;
        modelPoint -= _cm;
        H(0, 0) += dataPoint.x() * modelPoint.x();
        H(0, 1) += dataPoint.x() * modelPoint.y();
        H(0, 2) += dataPoint.x() * modelPoint.z();
        H(1, 0) += dataPoint.y() * modelPoint.x();
        H(1, 1) += dataPoint.y() * modelPoint.y();
        H(1, 2) += dataPoint.y() * modelPoint.z();
        H(2, 0) += dataPoint.z() * modelPoint.x();
        H(2, 1) += dataPoint.z() * modelPoint.y();
        H(2, 2) += dataPoint.z() * modelPoint.z();
    }

    auto a = H.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);

    R = a.matrixV() * a.matrixU().transpose();

    t = _cm - R*_cd;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform.block<3,3>(0,0) = R;
    transform.block<3,1>(0,3) = t;

    _transformation *= transform;

    //Setting new point cloud
    pcl::transformPointCloud(*_D, *_D, transform);

    for (auto point : *pairs) {
        error += std::powf(_M->points[point.modelPointIndex].x - _D->points[point.dataPointIndex].x, 2)
                 + std::powf(_M->points[point.modelPointIndex].y - _D->points[point.dataPointIndex].y, 2) +
                 +std::powf(_M->points[point.modelPointIndex].z - _D->points[point.dataPointIndex].z, 2);
    }
    error /= pairs->size();

    float diff = abs(oldError - error);

    delete pairs;
    return diff < eps;
};
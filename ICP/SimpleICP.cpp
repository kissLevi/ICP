//
// Created by Kiss Levente on 2019. 12. 17..
//

#include "SimpleICP.h"

void SimpleICP::run(int maxIterations, bool showResult, float eps){
    Eigen::Vector3f t = Eigen::Vector3f::Zero(3);
    Eigen::Matrix3f R = Eigen::Matrix3f::Ones();

    _error = 0;

    for(int i = 0; i < maxIterations; ++i)
    {
        if (iterate(R, t,_error,eps))
        {
            break;
        }

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform << R(0,0), R(0,1), R(0,2), t(0),
                R(1,0), R(1,1), R(1,2), t(1),
                R(2,0), R(2,1), R(2,2), t(2),
                0, 0, 0, 1;

        _transformation *= transform;

        //Setting new point cloud
        pcl::transformPointCloud(*_D, *_D, transform);

        if(showResult)
        {
            std::cout << "Iteration: " <<i+1 << std::endl;
            std::cout << "Rotation matrix: " << std::endl << R << std::endl;
            std::cout << "Translation vector: " << std::endl << t << std::endl;
            std::cout << "Error: " << _error << std::endl;
        }
    }
}

bool SimpleICP::iterate(
        Eigen::Matrix3f& R,
        Eigen::Vector3f& t,
        float& error,
        float eps) {

    //Number of nearest neighbours
    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);


    auto * pairs (new std::vector<struct PointPair>);

    for(auto i = 0; i < _D->size(); ++i)
    {
        if(_kdTree.nearestKSearch(_D->points[i],K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            pairs->push_back(PointPair(pointNKNSquaredDistance[0],pointIdxNKNSearch[0],i));
        }
    }

    float oldError = error;
    error = 0;

    _cm = Eigen::Vector3f::Zero();
    _cd = Eigen::Vector3f::Zero();
    for(auto & pair : *pairs)
    {
        _cm(0) += _M->points[ pair.modelPointIndex].x;
        _cm(1) += _M->points[ pair.modelPointIndex].y;
        _cm(2) += _M->points[ pair.modelPointIndex].z;

        _cd(0) += _D->points[ pair.dataPointIndex].x;
        _cd(1) += _D->points[ pair.dataPointIndex].y;
        _cd(2) += _D->points[ pair.dataPointIndex].z;

        error += std::powf(_M->points[ pair.modelPointIndex].x - _D->points[pair.dataPointIndex].x, 2)
                            + std::powf(_M->points[ pair.modelPointIndex].y - _D->points[pair.dataPointIndex].y, 2) +
                            + std::powf(_M->points[ pair.modelPointIndex].z - _D->points[pair.dataPointIndex].z, 2);
    }
    error /= _D->size();
    _cm /= pairs->size();
    _cd /= pairs->size();

    std::cout << pairs->size() << std::endl;


    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f dataPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f modelPoint = Eigen::Vector3f::Zero();


    for(auto & pair : *pairs) {

        dataPoint << _D->points[pair.dataPointIndex].x, _D->points[pair.dataPointIndex].y, _D->points[pair.dataPointIndex].z;
        modelPoint << _M->points[pair.modelPointIndex].x, _M->points[pair.modelPointIndex].y, _M->points[pair.modelPointIndex].z;
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

    float diff = abs(oldError - error);

    delete pairs;
    return diff < eps;
};
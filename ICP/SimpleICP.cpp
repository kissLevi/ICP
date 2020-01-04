//
// Created by Kiss Levente on 2019. 12. 17..
//

#include "SimpleICP.h"

void SimpleICP::run(int maxIterations,
         bool showResult,
         float eps){
    Eigen::Vector3d t = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d R = Eigen::Matrix3d::Ones();

    _error = 0;

    for(int i = 0; i < maxIterations; ++i)
    {
        if (iterate(R, t,_error,eps))
        {
            break;
        }


        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform << R(0,0), R(0,1), R(0,2), t(0),
                R(1,0), R(1,1), R(1,2), t(1),
                R(2,0), R(2,1), R(2,2), t(2),
                0, 0, 0, 1;

        Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();

        rotation << R(0,0), R(0,1), R(0,2), 0,
                R(1,0), R(1,1), R(1,2), 0,
                R(2,0), R(2,1), R(2,2), 0,
                0, 0, 0, 1;

        Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();

        translation << 1, 0, 0, t(0),
                0, 1, 0, t(1),
                0, 0, 1, t(2),
                0, 0, 0, 1;

        _transformation *= transform;

        //Setting new point cloud
        pcl::transformPointCloud(*_D, *_D, transform.transpose().cast<float>());

        setNewD(_D);

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
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t,
        float& error,
        float eps) {

    //Number of nearest neighbours
    const int K = 50;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    auto * pairs (new std::set<struct PointPair>);

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for(auto i = 0; i < _D->size(); ++i)
    {
        if(_kdTree.nearestKSearch(_D->points[i],K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            std::pair<std::set<struct PointPair>::iterator , bool > result;

            int elementIndex = 0;
            do{
                result = pairs->insert(PointPair(pointNKNSquaredDistance[elementIndex],pointIdxNKNSearch[elementIndex],i));
                ++elementIndex;
            }while(!result.second && elementIndex <K);
            elementIndex--;
            if(result.second && elementIndex >= 0) {
                /*Eigen::Vector3d point0;
                Eigen::Vector3d point1;

                point0 << _D->points[i].x, _D->points[i].y, _D->points[i].z;
                point1 <<   _M->points[ pointIdxNKNSearch[elementIndex] ].x,
                        _M->points[ pointIdxNKNSearch[elementIndex] ].y,
                        _M->points[ pointIdxNKNSearch[elementIndex] ].z;
                //Remove means
                point0 -= _cm;
                point1 -= _cd;
                H(0,0) += point0.x() *point1.x();
                H(0,1) += point0.x() *point1.y();
                H(0,2) += point0.x() *point1.z();
                H(1,0) += point0.y() *point1.x();
                H(1,1) += point0.y() *point1.y();
                H(1,2) += point0.y() *point1.z();
                H(2,0) += point0.z() *point1.x();
                H(2,1) += point0.z() *point1.y();
                H(2,2) += point0.z() *point1.z();*/
            }


        }
    }




    float oldError = error;
    error = 0;

    _cm = Eigen::Vector3d::Zero();
    _cd = Eigen::Vector3d::Zero();
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

    Eigen::Vector3d point0;
    Eigen::Vector3d point1;
    for(auto & pair : *pairs) {
        point0 << _D->points[pair.dataPointIndex].x, _D->points[pair.dataPointIndex].y, _D->points[pair.dataPointIndex].z;
        point1 << _M->points[pair.modelPointIndex].x,
                _M->points[pair.modelPointIndex].y,
                _M->points[pair.modelPointIndex].z;
        //Remove means
        point0 -= _cm;
        point1 -= _cd;
        H(0, 0) += point0.x() * point1.x();
        H(0, 1) += point0.x() * point1.y();
        H(0, 2) += point0.x() * point1.z();
        H(1, 0) += point0.y() * point1.x();
        H(1, 1) += point0.y() * point1.y();
        H(1, 2) += point0.y() * point1.z();
        H(2, 0) += point0.z() * point1.x();
        H(2, 1) += point0.z() * point1.y();
        H(2, 2) += point0.z() * point1.z();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);

    R = svd.matrixV() * svd.matrixU().transpose();

    t = _cm - R*_cd;

    float diff = abs(oldError - error);

    delete pairs;
    return diff < eps;
};
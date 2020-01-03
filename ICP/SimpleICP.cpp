//
// Created by Kiss Levente on 2019. 12. 17..
//

#include "SimpleICP.h"

void SimpleICP::run(int maxIterations,
         bool showResult,
         float eps){
    Eigen::Vector3d t = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

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

        //Setting new point cloud
        pcl::transformPointCloud(*_D, *_D, transform);
        setNewD(_D);

        _transformation *= transform;

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
    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::vector<int>* indexes (new std::vector<int>(_D->size()));

    Eigen::Matrix3d H;
    for(auto it = _D->begin(); it != _D->end(); ++it)
    {
        if(_kdTree.nearestKSearch(*it,K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            indexes->push_back(pointIdxNKNSearch[0]);

            Eigen::Vector3d point0;
            Eigen::Vector3d point1;

            point0 << it->x, it->y, it->z;
            point1 <<   _M->points[ pointIdxNKNSearch[0] ].x,
                    _M->points[ pointIdxNKNSearch[0] ].y,
                    _M->points[ pointIdxNKNSearch[0] ].z;
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
            H(2,2) += point0.z() *point1.z();
        }
    }
    std::cout << "H simple : " <<std::endl;
    std::cout << H <<std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);

    R = svd.matrixV() * svd.matrixU().transpose();

    t = _cm - R*_cd;

    float oldError = error;
    error = 0;
    int i = 0;
    for(auto it = _D->begin(); it != _D->end(); ++it)
    {

        error += std::powf(_M->points[ indexes->at(i) - it->x].x, 2)
                            + std::powf(_M->points[ indexes->at(i) ].y - it->y, 2) +
                            + std::powf(_M->points[ indexes->at(i) ].z - it->z, 2);
        ++i;
    }
    error /= _D->size();
    float diff = abs(oldError - error);

    delete indexes;
    return diff < eps;
};
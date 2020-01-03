//
// Created by Kiss Levente on 2019. 12. 20..
//

#include "TrICP.h"

void TrICP::run(int maxIterations, bool showResult, float eps) {
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

bool TrICP::iterate(
        Eigen::Matrix3d& R,
        Eigen::Vector3d& t,
        float& error,
        float eps){

    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    auto points(new std::vector<PointPair>());

    const int maxSize = _D->size()*_selectionPercentage;

    int counter = 0;

    for(int i = 0; i < _D->size(); ++i)
    {
        if(_kdTree.nearestKSearch(_D->at(i),K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            points->push_back(PointPair(pointNKNSquaredDistance[0],pointIdxNKNSearch[0],i));
            if(points->size() > maxSize)
            {
                auto it = --points->end();
                points->erase(it);
                counter--;
            }
        }
    }

    Eigen::Matrix3d H;

    //std::pair<Eigen::Vector3d, Eigen::Vector3d> means = _means(*points);

    for (auto & point : *points) {
        Eigen::Vector3d modelPoint;
        modelPoint << _M->at(point.modelPointIndex).x,
                _M->at(point.modelPointIndex).y,
                _M->at(point.modelPointIndex).z;
        Eigen::Vector3d dataPoint;
        modelPoint << _D->at(point.dataPointIndex).x,
                _D->at(point.dataPointIndex).y,
                _D->at(point.dataPointIndex).z;

        //Remove means
        modelPoint -= _cm;
        dataPoint -= _cd;
        H(0, 0) += modelPoint.x() * dataPoint.x();
        H(0, 1) += modelPoint.x() * dataPoint.y();
        H(0, 2) += modelPoint.x() * dataPoint.z();
        H(1, 0) += modelPoint.y() * dataPoint.x();
        H(1, 1) += modelPoint.y() * dataPoint.y();
        H(1, 2) += modelPoint.y() * dataPoint.z();
        H(2, 0) += modelPoint.z() * dataPoint.x();
        H(2, 1) += modelPoint.z() * dataPoint.y();
        H(2, 2) += modelPoint.z() * dataPoint.z();
    }
    std::cout << "tricp H: " << std::endl;
    std::cout << H <<std::endl;

    /*Eigen::Vector3d pointM = means.first;
    Eigen::Vector3d pointD = means.second;

    N(0,0) = pointM.x()*pointD.x() + pointM.y()*pointD.y() + pointM.z()*pointD.z();
    N(0,1) = pointM.y()*pointD.z() + pointM.z()*pointD.y();
    N(0,2) = pointM.z()*pointD.x() + pointM.x()*pointD.z();
    N(0,3) = pointM.x()*pointD.y() + pointM.y()*pointD.x();

    N(1,0) = pointM.y()*pointD.z() + pointM.z()*pointD.y();
    N(1,1) = pointM.x()*pointD.x() - pointM.y()*pointD.y() - pointM.z()*pointD.z();
    N(1,2) = pointM.x()*pointD.y() + pointM.y()*pointD.x();
    N(1,3) = pointM.z()*pointD.x() + pointM.x()*pointD.z();

    N(2,0) = pointM.z()*pointD.x() + pointM.x()*pointD.z();
    N(2,1) = pointM.x()*pointD.y() + pointM.y()*pointD.x();
    N(2,2) = -pointM.x()*pointD.x() + pointM.y()*pointD.y() - pointM.z()*pointM.z();
    N(2,3) = pointM.y()*pointD.z() + pointM.z()*pointD.y();

    N(3,0) = pointM.x()*pointD.y() + pointM.y()*pointD.x();
    N(3,1) = pointM.y()*pointD.z() + pointM.z()*pointD.y();
    N(3,2) = pointM.z()*pointD.x() + pointM.x()*pointD.z();
    N(3,3) = -pointM.x()*pointD.x() - pointM.y()*pointD.y() + pointM.z()*pointD.z();*/
    /*Eigen::EigenSolver<Eigen::Matrix4d> solver(N, true);

    /*std::cout << solver.eigenvectors().cast<double>() <<std::endl;
    std::cout << solver.eigenvalues().cast<double>() <<std::endl;*/

    /*Eigen::Vector4d eigenValues = solver.eigenvalues().real();

    double biggestEigenValueIndex =  0;
    for(int i = 1;i<4;++i)
    {
        if(eigenValues(biggestEigenValueIndex)<eigenValues(i))
        {
            biggestEigenValueIndex = i;
        }
    }

    Eigen::Vector4d greatestEigenVector = solver.eigenvectors().real().row(biggestEigenValueIndex);

    R(0,0) = std::powf(greatestEigenVector(0),2) + std::powf(greatestEigenVector(1),2) - std::powf(greatestEigenVector(2),2) - std::powf(greatestEigenVector(3),2);
    R(0,1) = 2*greatestEigenVector(1)*greatestEigenVector(2) - 2*greatestEigenVector(0)*greatestEigenVector(3);
    R(0,2) = 2*greatestEigenVector(1)*greatestEigenVector(3) + 2*greatestEigenVector(0)*greatestEigenVector(2);

    R(1,0) = 2*greatestEigenVector(0)*greatestEigenVector(3) + 2*greatestEigenVector(1)*greatestEigenVector(2);
    R(1,1) = std::powf(greatestEigenVector(0),2) - std::powf(greatestEigenVector(1),2) + std::powf(greatestEigenVector(2),2) - std::powf(greatestEigenVector(3),2);
    R(1,2) = 2*greatestEigenVector(2)*greatestEigenVector(3) - 2*greatestEigenVector(0)*greatestEigenVector(1);

    R(2,0) = 2*greatestEigenVector(1)*greatestEigenVector(3) - 2*greatestEigenVector(0)*greatestEigenVector(2);
    R(2,1) = 2*greatestEigenVector(0)*greatestEigenVector(1) + 2*greatestEigenVector(2)*greatestEigenVector(3);
    R(2,2) = std::powf(greatestEigenVector(0),2) - std::powf(greatestEigenVector(1),2) - std::powf(greatestEigenVector(2),2) + std::powf(greatestEigenVector(3),2);*/

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);

    R = svd.matrixV() * svd.matrixU().transpose();

    t = _cm - R* _cd;

    float oldError = error;
    error = 0;
    int i = 0;
    for(auto & point : *points)
    {
        error += std::sqrtf(std::powf(_D->at(point.dataPointIndex).x - _M->at(point.modelPointIndex).x, 2)
                            + std::powf(_D->at(point.dataPointIndex).y - _M->at(point.modelPointIndex).y, 2) +
                            + std::powf(_D->at(point.dataPointIndex).z - _M->at(point.modelPointIndex).z, 2));
        ++i;
    }

    float diff = abs(oldError - error);

    delete points;

    return diff < eps || error < _smallTrimemdMSE;

}
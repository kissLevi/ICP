//
// Created by Kiss Levente on 2019. 12. 20..
//

#include "TrICP.h"

void TrICP::run(int maxIterations, bool showResult, float eps) {
    Eigen::Vector3f t = Eigen::Vector3f::Zero(3);
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

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

bool TrICP::iterate(
        Eigen::Matrix3f& R,
        Eigen::Vector3f& t,
        float& error,
        float eps){

    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    auto points(new std::vector<struct PointPair>());

    const int maxSize = _D->size()*_selectionPercentage;

    float oldError = 0;

    _cm = Eigen::Vector3f::Zero();
    _cd = Eigen::Vector3f::Zero();


    for(int i = 0; i < _D->size(); ++i)
    {
        if(_kdTree.nearestKSearch(_D->points[i],K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            points->push_back(PointPair(pointNKNSquaredDistance[0],pointIdxNKNSearch[0],i));

            _cm(0) += _M->points[pointIdxNKNSearch[0]].x;
            _cm(1) += _M->points[pointIdxNKNSearch[0]].y;
            _cm(2) += _M->points[pointIdxNKNSearch[0]].z;

            _cd(0) += _D->points[ i].x;
            _cd(1) += _D->points[ i].y;
            _cd(2) += _D->points[ i].z;

            oldError += std::powf(_M->points[pointIdxNKNSearch[0]].x - _D->points[i].x, 2)
                     + std::powf(_M->points[pointIdxNKNSearch[0]].y - _D->points[i].y, 2) +
                     + std::powf(_M->points[pointIdxNKNSearch[0]].z - _D->points[i].z, 2);
        }
    }

    oldError /= _D->size();
    _cm /= points->size();
    _cd /= points->size();

    std::sort(points->begin(),points->end());


    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f modelPoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f dataPoint = Eigen::Vector3f::Zero();

    for (auto point : *points) {
        dataPoint << _D->at(point.dataPointIndex).x, _D->at(point.dataPointIndex).y, _D->at(point.dataPointIndex).z;
        modelPoint << _M->at(point.modelPointIndex).x, _M->at(point.modelPointIndex).y, _M->at(point.modelPointIndex).z;
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

    delete points;
    return diff < eps;
}


/*Eigen::Vector3f pointM = means.first;
   Eigen::Vector3f pointD = means.second;

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
/*Eigen::EigenSolver<Eigen::Matrix4f> solver(N, true);

/*std::cout << solver.eigenvectors().cast<double>() <<std::endl;
std::cout << solver.eigenvalues().cast<double>() <<std::endl;*/

/*Eigen::Vector4f eigenValues = solver.eigenvalues().real();

double biggestEigenValueIndex =  0;
for(int i = 1;i<4;++i)
{
    if(eigenValues(biggestEigenValueIndex)<eigenValues(i))
    {
        biggestEigenValueIndex = i;
    }
}

Eigen::Vector4f greatestEigenVector = solver.eigenvectors().real().row(biggestEigenValueIndex);

R(0,0) = std::powf(greatestEigenVector(0),2) + std::powf(greatestEigenVector(1),2) - std::powf(greatestEigenVector(2),2) - std::powf(greatestEigenVector(3),2);
R(0,1) = 2*greatestEigenVector(1)*greatestEigenVector(2) - 2*greatestEigenVector(0)*greatestEigenVector(3);
R(0,2) = 2*greatestEigenVector(1)*greatestEigenVector(3) + 2*greatestEigenVector(0)*greatestEigenVector(2);

R(1,0) = 2*greatestEigenVector(0)*greatestEigenVector(3) + 2*greatestEigenVector(1)*greatestEigenVector(2);
R(1,1) = std::powf(greatestEigenVector(0),2) - std::powf(greatestEigenVector(1),2) + std::powf(greatestEigenVector(2),2) - std::powf(greatestEigenVector(3),2);
R(1,2) = 2*greatestEigenVector(2)*greatestEigenVector(3) - 2*greatestEigenVector(0)*greatestEigenVector(1);

R(2,0) = 2*greatestEigenVector(1)*greatestEigenVector(3) - 2*greatestEigenVector(0)*greatestEigenVector(2);
R(2,1) = 2*greatestEigenVector(0)*greatestEigenVector(1) + 2*greatestEigenVector(2)*greatestEigenVector(3);
R(2,2) = std::powf(greatestEigenVector(0),2) - std::powf(greatestEigenVector(1),2) - std::powf(greatestEigenVector(2),2) + std::powf(greatestEigenVector(3),2);*/
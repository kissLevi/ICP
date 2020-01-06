//
// Created by Kiss Levente on 2020. 01. 06..
//

#include "MeasurementHelper.h"


void MeasurementHelper::rotateRandomly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix3f &rotation)
{
    double xr = ((double) rand() / (RAND_MAX)) * M_PI / 9;
    double yr = ((double) rand() / (RAND_MAX)) * M_PI / 9;
    double zr = ((double) rand() / (RAND_MAX)) * M_PI / 9;

    Eigen::Matrix3f xRotation = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f yRotation = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f zRotation = Eigen::Matrix3f::Identity();

    xRotation(1,1) = cos(xr);
    xRotation(1,2) = -sin(xr);
    xRotation(2,1) = sin(xr);
    xRotation(2,2) = cos(xr);

    yRotation(0,0) = cos(yr);
    yRotation(0,2) = sin(yr);
    yRotation(2,0) = -sin(yr);
    yRotation(2,2) = cos(yr);

    zRotation(0,0) = cos(zr);
    zRotation(0,1) = -sin(zr);
    zRotation(1,0) = sin(zr);
    zRotation(1,1) = cos(zr);

    std::cout << xr << std::endl;
    std::cout << yr << std::endl;
    std::cout << zr << std::endl;

    rotation = xRotation*yRotation*zRotation;

    Eigen::Matrix4f final = Eigen::Matrix4f::Identity();
    final.block<3,3>(0,0) = rotation;

    pcl::transformPointCloud(*cloud,*cloud,final);
}

void MeasurementHelper::translateRandomly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f &translation)
{
    double xt = ((double) rand() / (RAND_MAX)) * 1;
    double yt = ((double) rand() / (RAND_MAX)) * 1;
    double zt = ((double) rand() / (RAND_MAX)) * 1;

    translation(0) = xt;
    translation(1) = yt;
    translation(2) = zt;

    Eigen::Matrix4f final = Eigen::Matrix4f::Identity();
    final.block<3,1>(0,3) = translation;

    pcl::transformPointCloud(*cloud,*cloud,final);
}

void MeasurementHelper::addGaussianNoiseToRandomLocations(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mean, double stddev)
{
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    for(auto &pt : *cloud)
    {
        pt.x += dist(generator);
        pt.y += dist(generator);
        pt.z += dist(generator);
    }
}

void MeasurementHelper::removePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float percentage)
{
    int newNumberOfPoints = cloud->size() * percentage;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    std::set<int> pointsToRemove;

    do{
        int index = ((double) rand() / (RAND_MAX)) * newNumberOfPoints;
        auto inserted = pointsToRemove.insert(index);
        if(inserted.second)
        {
            inliers->indices.push_back(index);
        }
    }while(pointsToRemove.size() < newNumberOfPoints);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}

void MeasurementHelper::rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float rotationInDegrees, int axis)
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    float rotationRadians = rotationInDegrees / 180.0 * M_PI;
    if(axis ==0)
    {
        rotation(1,1) = cos(rotationRadians);
        rotation(1,2) = -sin(rotationRadians);
        rotation(2,1) = sin(rotationRadians);
        rotation(2,2) = cos(rotationRadians);

    }
    else if(axis == 1)
    {
        rotation(0,0) = cos(rotationRadians);
        rotation(0,2) = sin(rotationRadians);
        rotation(2,0) = -sin(rotationRadians);
        rotation(2,2) = cos(rotationRadians);
    }
    else
    {
        rotation(0,0) = cos(rotationRadians);
        rotation(0,1) = -sin(rotationRadians);
        rotation(1,0) = sin(rotationRadians);
        rotation(1,1) = cos(rotationRadians);
    }
    Eigen::Matrix4f final = Eigen::Matrix4f::Identity();
    final.block<3,3>(0,0) = rotation;

    pcl::transformPointCloud(*cloud,*cloud,final);
}

void MeasurementHelper::calculateRotations(Eigen::Matrix3f rotationMatrix, float &xrot, float &yrot, float &zrot)
{
    xrot = atan2( rotationMatrix(2,1),rotationMatrix(2,2) ) / M_PI * 180.0;
    yrot = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  ) / M_PI * 180.0;
    zrot = atan2( rotationMatrix(1,0),rotationMatrix(0,0) ) / M_PI * 180.0;
}

float MeasurementHelper::caluclateTranslationError(Eigen::Vector3f translation, Eigen::Vector3f calculatedTranslation)
{
    float error = 0;
    for(int i = 0;i< 3;++i)
    {
        error += abs(translation(i) - calculatedTranslation(i));
    }
    error /= 3;
    return error;
}
float MeasurementHelper::caluclateRotationalError(float x, float y, float z, Eigen::Matrix3f calculatedRotation)
{
    float calcRotx, calcRoty, calcRotz;
    MeasurementHelper::calculateRotations(calculatedRotation,calcRotx,calcRoty,calcRotz);
    float error = abs(-x-calcRotx) + abs(-y-calcRoty) + abs(-z-calcRotz);
    error /= 3;
    return error;
}
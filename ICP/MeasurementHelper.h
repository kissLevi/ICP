//
// Created by Kiss Levente on 2020. 01. 06..
//

#ifndef ICP_MEASUREMENTHELPER_H
#define ICP_MEASUREMENTHELPER_H


#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>
#include <random>
#include <set>


class MeasurementHelper {
public:
    static void rotateRandomly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix3f &rotation);
    static void rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float rotationInDegrees, int axis);
    static void translateRandomly(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f &translation);
    static void addGaussianNoiseToRandomLocations(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mean, double stddev);
    static void removePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float percentage);
    static void calculateRotations(Eigen::Matrix3f rotationMatrix, float &xrot, float &yrot, float &zrot);
    static float caluclateTranslationError(Eigen::Vector3f translation, Eigen::Vector3f calculatedTranslation);
    static float caluclateRotationalError(float x, float y, float z, Eigen::Matrix3f calculatedRotation);
};


#endif //ICP_MEASUREMENTHELPER_H

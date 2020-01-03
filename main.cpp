#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>
#include <vector>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/stuff/misc.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ICP/SimpleICP.h"
#include "ICP/TrICP.h"

bool iterate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr M,
             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr D,
             Eigen::Matrix3d& R,
             Eigen::Vector3d& t,
             float& error,
             bool showResult = false,
             float eps = 0.001);

Eigen::Vector3d mean(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr points);

int main(int argc, char** argv) {


    if(argc != 3) {
        std::cout << "Error! Requied parameters: Model point cloud,  Data point cloud." << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr M (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr P (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PLYReader reader;
    reader.read(argv[1],*M);
    reader.read(argv[2],*P);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,3) = 10.0;
    //transform_1 (0,3) = 5.0;

    pcl::transformPointCloud (*P, *P, transform_1);

    /*Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform << 1, 0, 0, 10,
                0, 1, 0, 3,
                0, 0, 1, 10,
                0, 0, 0, 1;
    pcl::transformPointCloud(*M,*P,transform);*/

    bool pclImpl = false;

    if(!pclImpl)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcpy (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*P,*pcpy);

        SimpleICP icp(M, pcpy);
        icp.run(1,false);

        pcl::copyPointCloud(*P,*pcpy);
        TrICP icp2(M, pcpy,1.0);
        icp2.run(1,false);


        std::cout << icp.getFinalTransformation() <<std::endl;

        //pcl::copyPointCloud(*icp.getTransformedD(),*P);
        //pcl::transformPointCloud(*P, *P, icp.getFinalTransformation());

        /*Eigen::Vector3d t = Eigen::Vector3d::Zero(3);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        float error;

        int maxNumberOfIterations = 50;



        for(int i = 0; i < maxNumberOfIterations; ++i)
        {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform << R(0,0), R(0,1), R(0,2), t(0),
                    R(1,0), R(1,1), R(1,2), t(1),
                    R(2,0), R(2,1), R(2,2), t(2),
                    0, 0, 0, 1;
            pcl::transformPointCloud(*P, *P, transform);
            if (iterate(M, P, R, t, error))
            {
                break;
            }

        }*/
    }
    else{
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(M);
        icp.setInputTarget(P);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        std::cout << icp.getFinalTransformation() << std::endl;

        pcl::transformPointCloud(*P, *P, icp.getFinalTransformation().inverse());
    }





    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (M, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZ> (P, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "sample cloud2");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}

bool iterate(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr M,
             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr D,
             Eigen::Matrix3d& R,
             Eigen::Vector3d& t,
             float& error,
             bool showResult,
             float eps) {

    //Calculate means of point clouds
    Eigen::Vector3d cm,cd;

    cm = mean(M);
    cd = mean(D);

    //Create Kd-tree of model points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (M);

    //Number of nearest neighbours
    const int K = 1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::vector<int>* indexes (new std::vector<int>(D->size()));

    Eigen::Matrix3d H;

    for(auto it = D->begin(); it != D->end(); ++it)
    {
        if(kdtree.nearestKSearch(*it,K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
        {
            indexes->push_back(pointIdxNKNSearch[0]);

            Eigen::Vector3d point0;
            Eigen::Vector3d point1;

            point0 << it->x, it->y, it->z;
            point1 <<   M->points[ pointIdxNKNSearch[0] ].x,
                    M->points[ pointIdxNKNSearch[0] ].y,
                    M->points[ pointIdxNKNSearch[0] ].z;
            //Remove means
            point0 -= cm;
            point1 -= cd;
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

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);

    R = svd.matrixV() * svd.matrixU().transpose();

    t = cm - R*cd;

    float oldError = error;
    error = 0;
    int i = 0;
    for(auto it = D->begin(); it != D->end(); ++it)
    {
        error += std::sqrtf(std::powf(it->x - M->points[ indexes->at(i) ].x, 2)
                + std::powf(it->y - M->points[ indexes->at(i) ].y, 2) +
                + std::powf(it->z - M->points[ indexes->at(i) ].z, 2));
        ++i;
    }
    error /= D->size();
    float diff = abs(oldError - error);
    if(showResult) {
        std::cout << "Rotation " << std::endl << R << std::endl;
        std::cout << "Translation " << std::endl << t << std::endl;
        std::cout << "Error "<< error << std::endl;
        std::cout << oldError << std::endl;
        std::cout << "Error difference " << diff << std::endl;
    }


    delete indexes;
    return diff < eps;
}

Eigen::Vector3d mean(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr points) {
    Eigen::Vector3d cm;

    for(auto it = points->begin(); it != points->end(); ++it){

        cm(0) += it->x;
        cm(1) += it->y;
        cm(2) += it->z;
    }
    cm /= points->size();
    return cm;
}
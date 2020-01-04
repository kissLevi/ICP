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


    if(argc != 5) {
        std::cout << "Error! Requied parameters: Model point cloud,  Data point cloud. Plc implementation." << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr M (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr P (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PLYReader reader;
    reader.read(argv[1],*M);
    reader.read(argv[2],*P);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,3) = 5.0;
    pcl::transformPointCloud (*P, *P, transform_1);


    pcl::copyPointCloud(*P,*Final);


    bool pclImpl = std::stoi(argv[3]);
    int maxIterations = std::stoi(argv[4]);

    if(!pclImpl)
    {
        SimpleICP icp(M, Final);
        icp.run(maxIterations,false,0.000001);

        auto finalTransformation = icp.getFinalTransformation();

        std::cout << finalTransformation <<std::endl;

    }
    else{

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(P);
        icp.setInputTarget(M);
        icp.setMaximumIterations(maxIterations);
        icp.align(*Final);
        std::cout << icp.getFinalTransformation() << std::endl;

    }

    //Visualize result
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (M, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZ> (P, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "sample cloud2");
    viewer->addPointCloud<pcl::PointXYZ> (Final, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1, "sample cloud3");

    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}

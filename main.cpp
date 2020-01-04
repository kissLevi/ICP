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


    if(argc != 4) {
        std::cout << "Error! Requied parameters: Model point cloud,  Data point cloud. Plc implementation." << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr M (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr P (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr origP (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcpy (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PLYReader reader;
    reader.read(argv[1],*M);
    reader.read(argv[2],*P);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,3) = 20.0;
    pcl::transformPointCloud (*P, *P, transform_1);


    pcl::copyPointCloud(*P,*origP);


    bool pclImpl = std::stoi(argv[3]);

    if(!pclImpl)
    {

        pcl::copyPointCloud(*P,*pcpy);

        SimpleICP icp(M, pcpy);
        icp.run(1,true);

        auto finalTransformation = icp.getFinalTransformation();

        std::cout << finalTransformation <<std::endl;

        for(auto &pt : *P) {
            Eigen::Vector4d tmp;

            tmp << pt.x, pt.y, pt.z, 1;

            tmp = finalTransformation * tmp;

            pt.x = tmp(0) / tmp(3);
            pt.y = tmp(1) / tmp(3);
            pt.z = tmp(2) / tmp(3);


        }
    }
    else{

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(M);
        icp.setInputTarget(P);
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);
        std::cout << icp.getFinalTransformation() << std::endl;
        pcl::transformPointCloud(*P, *P, icp.getFinalTransformation().inverse());

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

    viewer->addPointCloud<pcl::PointXYZ> (pcpy, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1, "sample cloud3");

    viewer->addPointCloud<pcl::PointXYZ> (origP, "sample cloud4");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,1, "sample cloud4");

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}

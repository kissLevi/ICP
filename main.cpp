#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include "ICP/SimpleICP.h"
#include "ICP/TrICP.h"
#include "ICP/MeasurementHelper.h"



#include <chrono>



int main(int argc, char** argv) {

    if(argc != 3) {
        std::cout << "Error! Requied parameters: input point cloud, max number of iterations." << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr M (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr P (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclResult (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr simpleIcpResult (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trIcpResult (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PLYReader reader;
    reader.read(argv[1],*M);
    reader.read(argv[1],*P);

    Eigen::Matrix3f rotation;
    MeasurementHelper::addGaussianNoiseToRandomLocations(P,0.0,0.001);
    MeasurementHelper::removePoints(P, 0.3);
    Eigen::Vector3f tr;

    float rotationX = 20;
    float rotationY = 0;
    float rotationZ = 0;

    float translationX = 0;
    float translationY = 0;
    float translationZ = 0;

    MeasurementHelper::rotate(P,rotationX,0);
    MeasurementHelper::rotate(P,rotationY,1);
    MeasurementHelper::rotate(P,rotationZ,2);

    pcl::copyPointCloud(*P,*pclResult);
    pcl::copyPointCloud(*P,*simpleIcpResult);
    pcl::copyPointCloud(*P,*trIcpResult);

    int maxIterations = std::stoi(argv[2]);


    auto begin = std::chrono::steady_clock::now();
    SimpleICP icp(M, simpleIcpResult);
    icp.run(maxIterations,false,0.0000001);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Time difference (sec) = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1000000.0 <<std::endl;
    std::cout << "Iterations taken " << icp.getNumberOfIterationTaken() << std::endl;
    auto finalTransformation = icp.getFinalTransformation();
    std::cout << finalTransformation <<std::endl;
    std:: cout << "Angular error " << MeasurementHelper::caluclateRotationalError(rotationX,rotationY,rotationZ, finalTransformation.block<3,3>(0,0)) << std::endl;

    begin = std::chrono::steady_clock::now();
    TrICP trIcp(M, trIcpResult, 0.7);
    trIcp.run(maxIterations,false,0.0000001);
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference (sec) = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1000000.0 <<std::endl;
    std::cout << "Iterations taken " << trIcp.getNumberOfIterationTaken() << std::endl;
    finalTransformation = trIcp.getFinalTransformation();
    std::cout << finalTransformation <<std::endl;
    std:: cout << "Angular error " << MeasurementHelper::caluclateRotationalError(rotationX,rotationY,rotationZ, finalTransformation.block<3,3>(0,0)) << std::endl;

    begin = std::chrono::steady_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icpPcl;
    icpPcl.setInputSource(P);
    icpPcl.setInputTarget(M);
    icpPcl.setMaximumIterations(maxIterations);
    icpPcl.align(*pclResult);
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference (sec) = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1000000.0 <<std::endl;
    finalTransformation = icpPcl.getFinalTransformation();
    std::cout << icpPcl.getFinalTransformation() << std::endl;
    std:: cout << "Angular error " << MeasurementHelper::caluclateRotationalError(rotationX,rotationY,rotationZ, finalTransformation.block<3,3>(0,0)) << std::endl;



    //Visualize result
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    //RED
    viewer->addPointCloud<pcl::PointXYZ> (M, "Model cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "Model cloud");

    //GREEN
    viewer->addPointCloud<pcl::PointXYZ> (P, "Data cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Data cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "Data cloud");

    //YELLOW
    viewer->addPointCloud<pcl::PointXYZ> (simpleIcpResult, "Simple cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Simple cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,0, "Simple cloud");

    //CYAN
    viewer->addPointCloud<pcl::PointXYZ> (trIcpResult, "Tr cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Tr cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,1, "Tr cloud");

    //BLUE
    viewer->addPointCloud<pcl::PointXYZ> (pclResult, "Pcl cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Pcl cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1, "Pcl cloud");

    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();

    viewer->setBackgroundColor (0, 0, 0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    return 0;
}



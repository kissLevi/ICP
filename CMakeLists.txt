cmake_minimum_required(VERSION 3.15)
project(icp)

set(CMAKE_CXX_STANDARD 14)

include_directories(/usr/local/opt/nanoflann/include)
link_directories(/usr/local/opt/nanoflann/lib)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#PCL
find_package(PCL 1.3 REQUIRED COMPONENTS io filters segmentation visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#G2o

include_directories(/usr/local/opt/g2o/include)
link_directories(/usr/local/opt/g2o/lib)

add_executable(icp main.cpp ICP/BaseICP.h ICP/SimpleICP.cpp ICP/SimpleICP.h ICP/TrICP.cpp ICP/TrICP.h)

target_link_libraries( icp  Eigen3::Eigen ${PCL_LIBRARIES}
        g2o_core
        g2o_types_slam3d
        g2o_solver_csparse
        g2o_stuff
        g2o_csparse_extension
        g2o_types_sba
        g2o_types_icp)
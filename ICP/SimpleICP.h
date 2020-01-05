//
// Created by Kiss Levente on 2019. 12. 17..
//

#ifndef ICP_SIMPLEICP_H
#define ICP_SIMPLEICP_H

#include "BaseICP.h"

class SimpleICP : public BaseICP  {
public:
    SimpleICP(BaseICP::PointCloud::Ptr M, BaseICP::PointCloud::Ptr D) :BaseICP(M,D){};
    void run(int maxIterations = 50,
             bool showResult = false,
             float eps = 0.001);
private:
    bool iterate(
            Eigen::Matrix3d& R,
            Eigen::Vector3d& t,
            float& error,
            float eps = 0.001);
};


#endif //ICP_SIMPLEICP_H

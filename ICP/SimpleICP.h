//
// Created by Kiss Levente on 2019. 12. 17..
//

#ifndef ICP_SIMPLEICP_H
#define ICP_SIMPLEICP_H

#include "BaseICP.h"

class SimpleICP : public BaseICP  {
public:
    SimpleICP(BaseICP::PointCloud::Ptr M, BaseICP::PointCloud::Ptr D) :BaseICP(M,D){};

protected:
    bool iterate(
            Eigen::Matrix3f& R,
            Eigen::Vector3f& t,
            float& error,
            float eps = 0.001);
};


#endif //ICP_SIMPLEICP_H

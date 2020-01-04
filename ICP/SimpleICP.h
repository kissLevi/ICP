//
// Created by Kiss Levente on 2019. 12. 17..
//

#ifndef ICP_SIMPLEICP_H
#define ICP_SIMPLEICP_H

#include "BaseICP.h"
#include <set>
#include <pcl/visualization/pcl_visualizer.h>

struct PointPair {

    float squaredDistance;
    int modelPointIndex;
    int dataPointIndex;

    PointPair(float squaredDistance, int modelPointIndex, int dataPointIndex):
        squaredDistance(squaredDistance),
        modelPointIndex(modelPointIndex),
        dataPointIndex(dataPointIndex){};
    PointPair() = default;

    bool operator <(const PointPair& pt) const
    {
        return (this->squaredDistance < pt.squaredDistance);
    }
};

/*if(squaredDistance == pt.squaredDistance)
{
    if(dataPointIndex == pt.dataPointIndex)
    {
        if(modelPointIndex == pt.modelPointIndex){
            return false;
        }else{
            return modelPointIndex <pt.modelPointIndex;
        }
    }
    else{
        return dataPointIndex <pt.dataPointIndex;
    }
}
else{
    return squaredDistance <pt.squaredDistance;
}*/

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

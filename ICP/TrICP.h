//
// Created by Kiss Levente on 2019. 12. 20..
//

#ifndef ICP_TRICP_H
#define ICP_TRICP_H

#include "BaseICP.h"
#include <algorithm>
#include <vector>
#include "SimpleICP.h"
#include <random>

class TrICP : public BaseICP {
public:
    TrICP(BaseICP::PointCloud::Ptr M,
            BaseICP::PointCloud::Ptr D,
            float selectionPercentage = 0.8,
            float smallTrimemdMSE = 0) :BaseICP(M,D),
                                        _selectionPercentage(selectionPercentage),
                                        _Sts(MAXFLOAT),
                                         _smallTrimemdMSE(smallTrimemdMSE){};
    void run(int maxIterations = 50,
             bool showResult = false,
             float eps = 0.001);

private:
    float _selectionPercentage;
    float _Sts;
    float _smallTrimemdMSE;

    bool iterate(
            Eigen::Matrix3f& R,
            Eigen::Vector3f& t,
            float& error,
            float eps = 0.001);
};


#endif //ICP_TRICP_H

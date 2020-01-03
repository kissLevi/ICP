//
// Created by Kiss Levente on 2019. 12. 20..
//

#ifndef ICP_TRICP_H
#define ICP_TRICP_H

#include "BaseICP.h"
#include <set>
#include <vector>
#include "SimpleICP.h"

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
            Eigen::Matrix3d& R,
            Eigen::Vector3d& t,
            float& error,
            float eps = 0.001);


    inline std::pair<Eigen::Vector3d, Eigen::Vector3d> _means(const std::multiset<PointPair>& points) {
        Eigen::Vector3d cm;
        Eigen::Vector3d cd;

        for(auto it = points.begin(); it != points.end(); ++it){
            Eigen::Vector3d tmpCm;
            Eigen::Vector3d tmpCd;

            tmpCm << _M->at(it->modelPointIndex).x, _M->at(it->modelPointIndex).y, _M->at(it->modelPointIndex).z;
            tmpCd << _D->at(it->dataPointIndex).x, _D->at(it->dataPointIndex).y, _D->at(it->dataPointIndex).z;
            cm += tmpCm;
            cd += tmpCd;
        }
        cm /= points.size();
        cd /= points.size();
        return std::pair<Eigen::Vector3d,Eigen::Vector3d>(cm,cd);
    }
};


#endif //ICP_TRICP_H

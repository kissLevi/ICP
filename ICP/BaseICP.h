//
// Created by Kiss Levente on 2019. 12. 12..
//

#ifndef ICP_BASEICP_H
#define ICP_BASEICP_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl-1.9/pcl/common/transforms.h>

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

    class BaseICP {

    public:
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        inline BaseICP(BaseICP::PointCloud::Ptr M, BaseICP::PointCloud::Ptr D) : _D(D), _M(M), _transformation(Eigen::Matrix4f::Identity())
        {
            _buildKdTree(M);
        _cm = _mean(M);
        _cd = _mean(D);
    };
    inline void setNewM(PointCloud::Ptr M)
    {
        _buildKdTree(M);
        _M = M;
        _cm = _mean(M);
    };
    inline void setNewD(PointCloud::Ptr D)
    {
        _D = D;
        _cd = _mean(D);
    };

    inline const Eigen::Matrix4f getFinalTransformation()
    {
        return _transformation;
    }

    inline const PointCloud::ConstPtr getTransformedD()
    {
        return _D;
    }
    virtual void run(int maxIterations = 50,
                     bool showResult = false,
                     float eps = 0.001) = 0;
protected:
    PointCloud::Ptr _D;
    PointCloud::Ptr _M;

    Eigen::Vector3f _cm;
    Eigen::Vector3f _cd;

    pcl::KdTreeFLANN<pcl::PointXYZ> _kdTree;

    float _error;
    Eigen::Matrix4f _transformation;

    inline Eigen::Vector3f _mean(const PointCloud::ConstPtr points) {
        Eigen::Vector3f cm;

        for(auto it = points->begin(); it != points->end(); ++it){
            cm(0) += it->x;
            cm(1) += it->y;
            cm(2) += it->z;
        }
        cm /= points->size();
        return cm;
    }
private:
    inline void _buildKdTree(PointCloud::Ptr points) {
        _kdTree.setInputCloud(points);
    }
};


#endif //ICP_BASEICP_H

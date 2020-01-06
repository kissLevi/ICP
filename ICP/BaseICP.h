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
        _cm = mean(M->begin(),M->end());
        _cd = mean(M->begin(),M->end());
    };
    inline void setNewM(PointCloud::Ptr M)
    {
        _buildKdTree(M);
        _M = M;
        _cm = mean(M->begin(),M->end());
    };
    inline void setNewD(PointCloud::Ptr D)
    {
        _D = D;
        _cd = mean(D->begin(),D->end());
    };

    inline const Eigen::Matrix4f getFinalTransformation()
    {
        return _transformation;
    }

    inline const PointCloud::ConstPtr getTransformedD()
    {
        return _D;
    }

    inline const int getNumberOfIterationTaken()
    {
        return _iterationTaked;
    }

    inline const float getFinalError()
    {
        return _finalError;
    }

    inline void run(int maxIterations, bool showResult, float eps) {
        Eigen::Vector3f t = Eigen::Vector3f::Zero(3);
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

        _finalError = 0;
        _iterationTaked = 1;

        for(int i = 0; i < maxIterations; ++i)
        {
            _iterationTaked++;
            if (iterate(R, t,_finalError,eps))
            {
                break;
            }

            if(showResult)
            {
                std::cout << "Iteration: " <<i+1 << std::endl;
                std::cout << "Rotation matrix: " << std::endl << R << std::endl;
                std::cout << "Translation vector: " << std::endl << t << std::endl;
                std::cout << "Error: " << _finalError << std::endl;
            }
        }
    }
protected:
    PointCloud::Ptr _D;
    PointCloud::Ptr _M;

    Eigen::Vector3f _cm;
    Eigen::Vector3f _cd;

    pcl::KdTreeFLANN<pcl::PointXYZ> _kdTree;

    float _error;
    Eigen::Matrix4f _transformation;

    virtual bool iterate(Eigen::Matrix3f& R, Eigen::Vector3f& t, float& error, float eps) = 0;

    template <typename iterator>
    inline Eigen::Vector3f mean(iterator first, iterator last)
    {
        int count = 0;
        Eigen::Vector3f result;
        for(iterator it = first; it!= last; it++)
        {
            pcl::PointXYZ z;
            count++;
            result += it->getVector3fMap();
        }
        return result /= count;
    }

private:

        int _iterationTaked;
    float _finalError;

    inline void _buildKdTree(PointCloud::Ptr points) {
        _kdTree.setInputCloud(points);
    }
};


#endif //ICP_BASEICP_H

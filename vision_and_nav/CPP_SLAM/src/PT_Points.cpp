#include "PT_Points.hpp"

static Eigen::Vector3d __PT_ToFromHomog(cv::Mat point);
static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat4d point);

struct PointSet* PT_InitPoints()
{
    struct PointSet* pts = (struct PointSet*) malloc(sizeof(struct PointSet));
    pts->points = {};
    pts->observations_indexes = {};
    return pts;
}

void PT_AddPoints(struct PointSet* pointset, cv::Mat4d points)
{
    const u32 Np = points.cols;
    for(u32 i = 0; i < Np; i++)
    {
        cv::Mat X = points.col(i);
        __PT_AddPoint(pointset, X);
    }
}

void PT_AddObs(struct PointSet* pointset, u64 pointidx, u64 obsidx)
{
    pointset->observations_indexes[pointidx].push_back(obsidx);
}

static Eigen::Vector3d __PT_ToFromHomog(cv::Mat point)
{
    const double EPS = 1 / 1000000.0f;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if(point.at<float>(3) < EPS) return v;
    v << static_cast<double>(point.at<float>(0)/point.at<float>(3)),
         static_cast<double>(point.at<float>(1)/point.at<float>(3)),
         static_cast<double>(point.at<float>(2)/point.at<float>(3));
    return v;
}

#define AT_INF(v) (v.isZero())

static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat4d point)
{
    Eigen::Vector3d v = __PT_ToFromHomog(point);
    //Do not add points at infinity
    if(AT_INF(v)) return;
    pointset->points.push_back(v);
    pointset->observations_indexes.push_back({});
}

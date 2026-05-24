#include "../include/PT_Points.hpp"
#include "PROJ_ProjectiveUtils.hpp"

static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat point);

struct PointSet* PT_InitPoints()
{
    struct PointSet* pts = new struct PointSet{};
    pts->points = {};
    pts->observations_indexes = {};
    pts->last_sz = 0;
    return pts;
}

void PT_AddPoints(struct PointSet* pointset, cv::Mat points)
{
    const u32 Np = points.cols;
    pointset->last_sz = pointset->points.size();
    for(u32 i = 0; i < Np; i++)
    {
        cv::Mat X = points.col(i);
        __PT_AddPoint(pointset, X);
    }
}

void PT_AddObs(struct PointSet* pointset, u64 pointidx, u64 obsidx)
{
    assert(pointset->observations_indexes.size() > pointset->last_sz + pointidx);
    pointset->observations_indexes[pointset->last_sz + pointidx].push_back(obsidx);
}

void PT_Print(struct PointSet* points)
{
    LG_Log("PointSet\n");
    LG_Log("points.size(): %lld\n", points->points.size());
    LG_Log("observations_indexes.size(): %lld\n", points->observations_indexes.size());
    LG_Log("last_sz: %lld\n", points->last_sz);

    size_t n = std::min<size_t>(points->points.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector4d& p = points->points[i];
        LG_Log("  point[%zu] = (%f, %f, %f)\n", i, p.x(), p.y(), p.z());
    }
}


static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat point)
{
    Eigen::Vector4d X = PROJ_CV2NormalizedEigen(point);
    pointset->points.push_back(X);
    pointset->observations_indexes.push_back({});
}


#include "../include/PT_Points.hpp"

static Eigen::Vector3d __PT_ToFromHomog(cv::Mat point);
static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat point);

struct PointSet* PT_InitPoints()
{
    struct PointSet* pts = (struct PointSet*) malloc(sizeof(struct PointSet));
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
        LG_Log("  point[%lld] ", i);
        if (!points->points[i].empty())
        {
            LG_Log("shape=(%lluX%llu)", points->points[i].rows, points->points[i].cols);
        }
        else
        {
            LG_Log("empty");
        }
        LG_Log("\n");
        //std::cout << " obsidx = (" << points->observations_indexes[i][0] << ", " << points->observations_indexes[i][1] << ")";
        //std::cout << ", obs count=" << points->observations_indexes[i].size() << "\n";
    }
}

static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat point)
{
    pointset->points.push_back(point);
    pointset->observations_indexes.push_back({});
}


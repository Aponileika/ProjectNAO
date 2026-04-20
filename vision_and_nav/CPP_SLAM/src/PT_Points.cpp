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
    std::cout << "PointSet\n";
    std::cout << "  points.size(): " << points->points.size() << "\n";
    std::cout << "  observations_indexes.size(): " << points->observations_indexes.size() << "\n";
    std::cout << "  last_sz: " << points->last_sz << "\n";

    size_t n = std::min<size_t>(points->points.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << "  point[" << i << "] ";
        if (!points->points[i].empty())
        {
            std::cout << "shape=(" << points->points[i].rows
                      << "x" << points->points[i].cols << ")";
        }
        else
        {
            std::cout << "empty";
        }
        std::cout << " obsidx = (" << points->observations_indexes[i][0] << ", " << points->observations_indexes[i][1] << ")";
        std::cout << ", obs count=" << points->observations_indexes[i].size() << "\n";
    }
}

static inline void __PT_AddPoint(struct PointSet* pointset, cv::Mat point)
{
    pointset->points.push_back(point);
    pointset->observations_indexes.push_back({});
}


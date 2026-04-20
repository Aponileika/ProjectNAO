#include "../include/OB_Observations.hpp"

void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index);

struct ObservationSet* OB_InitObs()
{
    struct ObservationSet* obs = (struct ObservationSet*)malloc(sizeof(struct ObservationSet));
    obs->observations = {};
    obs->view_indexes = {};
    obs->point_indexes = {};
    return obs;
}

void OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points, PointPair2D corrp)
{
    std::vector<cv::Point2d> img1points = corrp.first;
    std::vector<cv::Point2d> img2points = corrp.second;
    size_t num_points = img1points.size();
    size_t num_points2 = img2points.size();
    assert(num_points == num_points2);
    for(size_t i = 0; i < num_points; i++)
    {
        __OB_AddObs(obs, views, points, img1points[i], 0, i);
        __OB_AddObs(obs, views, points, img2points[i], 1, i);
    }
}

void OB_Print(struct ObservationSet* obs)
{
    std::cout << "ObservationSet\n";
    std::cout << "  observations.size(): " << obs->observations.size() << "\n";
    std::cout << "  view_indexes.size(): " << obs->view_indexes.size() << "\n";
    std::cout << "  point_indexes.size(): " << obs->point_indexes.size() << "\n";

    size_t n = std::min<size_t>(obs->observations.size(), 10);
    for (size_t i = 0; i < n; ++i)
    {
        std::cout << "  obs[" << i << "] = (" 
                  << obs->observations[i].x << ", "
                  << obs->observations[i].y << "), view="
                  << obs->view_indexes[i] << ", point="
                  << obs->point_indexes[i] << "\n";
    }
}

#define PUSHVIEW(vidx, views, idx)\
    (vidx.push_back(views->last_sz - 1 + idx))

#define PUSHPOINT(pidx, points, idx)\
    (pidx.push_back(points->last_sz + idx))

void __OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        cv::Point2d observation, u64 view_index, u64 point_index)
{
    obs->observations.push_back(observation);
    PUSHVIEW(obs->view_indexes, views, view_index);
    PUSHPOINT(obs->point_indexes, points, point_index);
    VW_AddObs(views, view_index, (u64)obs->observations.size() - 1);
    PT_AddObs(points, point_index, (u64)obs->observations.size() - 1);
}

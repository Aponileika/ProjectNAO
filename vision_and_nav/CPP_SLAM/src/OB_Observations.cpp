#include "OB_Observations.hpp"

struct ObservationSet* OB_InitObservations()
{
    struct ObservationSet* obs = (struct ObservationSet*)malloc(sizeof(struct ObservationSet));
    obs->observations = {};
    obs->view_indexes = {};
    obs->point_indexes = {};
    return obs;
}

void OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points,
        Eigen::Vector2d observation, u64 view_index, u64 point_index)
{
    obs->observations.push_back(observation);
    obs->view_indexes.push_back(view_index);
    obs->point_indexes.push_back(view_index);
    VW_AddObs(views, view_index, (u64)obs->observations.size());
    PT_AddObs(points, point_index, (u64)obs->observations.size());
}

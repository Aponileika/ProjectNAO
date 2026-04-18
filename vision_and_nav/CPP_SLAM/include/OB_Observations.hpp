#ifndef __OB__OBSERVATIONS_HPP
#define __OB__OBSERVATIONS_HPP
#include <vector>
#include <Eigen/Dense>
#include "CArenaAlloc.h"
#include "VW_Views.hpp"
#include "PT_Points.hpp"

struct ObservationSet
{
    std::vector<Eigen::Vector2d> observations;
    std::vector<u64> view_indexes;
    std::vector<u64> point_indexes;
};

struct ObservationSet* OB_InitObs();
void OB_AddObs(struct ObservationSet* obs, Eigen::Vector2d observation, u64 view_index, u64 point_index);

#endif //__OB__OBSERVATIONS_HPP

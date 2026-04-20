#ifndef __OB__OBSERVATIONS_HPP
#define __OB__OBSERVATIONS_HPP
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "CArenaAlloc.h"
#include "VW_Views.hpp"
#include "PT_Points.hpp"
#include "EP_CorrespondingPoints.hpp"

struct ObservationSet
{
    std::vector<cv::Point2d> observations;
    std::vector<u64> view_indexes;
    std::vector<u64> point_indexes;
};

struct ObservationSet* OB_InitObs();
void OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points, PointPair2D corrp);
void OB_Print(struct ObservationSet* obs);

#endif //__OB__OBSERVATIONS_HPP

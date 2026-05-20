#ifndef __OB__OBSERVATIONS_HPP
#define __OB__OBSERVATIONS_HPP
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"
#include "VW_Views.hpp"
#include "PT_Points.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "LG_Logging.hpp"

#define PnPRansacIts 800
#define Reprojerr 3.0f
#define conf 0.99f
#define PnPPointCntThreshold 10 
#define NonPnpThreshold 30 

struct ObservationSet
{
    std::vector<cv::Point2d> observations;
    std::vector<u64> view_indexes;
    std::vector<u64> point_indexes;
    //Maps 2D image point paired with view to idx
    std::map<std::pair<std::pair<fp64, fp64>, u64>, u64> imagepoint2idx;
};

typedef enum
{
    PNP_SUCCESS = 0,
    PNP_NOT_ENOUGH_2D3D = 1,
    PNP_NOT_ENOUGH_NONPNP = 2,
}PnPRet_t;

struct PnPret
{
    PointPair2D nonpnpPoints;
    PnPRet_t ret;
};

struct ObservationSet* OB_InitObs();
void OB_AddObs(struct ObservationSet* obs, struct ViewSet* views, struct PointSet* points, PointPair2D corrp);
void OB_Print(struct ObservationSet* obs);
struct PnPret OB_SolvePnP(PointPair2D corrp, ViewSet* TView, ObservationSet* TObs, PointSet* TPoints);

#endif //__OB__OBSERVATIONS_HPP

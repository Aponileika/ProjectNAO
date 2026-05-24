#ifndef __PT__Points_HPP_
#define __PT__Points_HPP_
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "CArenaAlloc.h"
#include "LG_Logging.hpp"
#include "PROJ_ProjectiveUtils.hpp"

struct PointSet
{
    /*
     * As suggested in:
     * Triggs, McLauchlan, Hartley, Fitzgibbon,
     * Bundle Adjustment – A Modern Synthesis,
     * Vision Algorithms: Theory and Practice, 2000
     * We keep the points in homogenous coordinates, and always normalize
     * them to have squared distance of 1 to the origin
     * */
    std::vector<Eigen::Vector4d> points;
    size_t last_sz;
    std::vector<std::vector<u64>> observations_indexes;
};

struct PointSet* PT_InitPoints();
void PT_AddPoints(struct PointSet* pointset, cv::Mat points);
void PT_AddObs(struct PointSet* points, u64 pointidx, u64 obsidx);
void PT_Print(struct PointSet* points);

#endif //__PT__Points_HPP_

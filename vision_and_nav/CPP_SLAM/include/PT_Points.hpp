#ifndef __PT__Points_HPP_
#define __PT__Points_HPP_
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "CArenaAlloc.h"
#include "LG_Logging.hpp"

struct PointSet
{
    std::vector<cv::Mat> points;
    size_t last_sz;
    std::vector<std::vector<u64>> observations_indexes;
};

struct PointSet* PT_InitPoints();
void PT_AddPoints(struct PointSet* pointset, cv::Mat points);
void PT_AddObs(struct PointSet* points, u64 pointidx, u64 obsidx);
void PT_Print(struct PointSet* points);
#endif //__PT__Points_HPP_

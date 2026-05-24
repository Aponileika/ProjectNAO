#ifndef __VW_VIEW_HPP_
#define __VW_VIEW_HPP_
#include <iostream>
#include <vector>
#include <unordered_map>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"

struct ViewSet
{
    std::vector<Camera> views;
    //store descriptors!
    std::vector<cv::Mat> descriptors;
    size_t last_sz;
    std::vector<std::vector<u64>> observations_indexes;
};

struct ViewSet* VW_InitViewSet();
void VW_AddView(struct ViewSet* views, Camera cam);
void VW_AddObs(struct ViewSet* views, u64 viewidx, u64 obsidx);
void VW_Print(struct ViewSet* views);
struct Camera* VW_GetTwoLatestCams(struct ViewSet* views);

#endif //__VW_VIEW_HPP_

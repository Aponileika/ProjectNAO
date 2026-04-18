#ifndef __VW_VIEW_HPP_
#define __VW_VIEW_HPP_
#include <vector>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"

struct ViewSet
{
    std::vector<Camera> views;
    std::vector<std::vector<u64>> observations_indexes;
};

struct ViewSet* VW_InitViewSet();
void VW_AddView(struct ViewSet* views, Camera cam);
void VW_AddObs(struct ViewSet* views, u64 viewidx, u64 obsidx);

#endif //__VW_VIEW_HPP_

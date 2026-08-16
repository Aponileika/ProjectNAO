#ifndef __GRAPH_PANTOGRAPH_HPP__
#define __GRAPH_PANTOGRAPH_HPP__
#include "CArenaAlloc.h"
#include <vector>
#include "KEY_Keyframe.hpp"
#include <map>
#include <algorithm>
#include "MAP_Mapping.hpp"
#include "PT_Types.hpp"

typedef struct
{
    u64 KeyFrameID;
    u64 Covisibility;
}typeCovisibility;

typedef std::vector<std::vector<typeCovisibility>> typeCovisibilityGraph;

void GRAPH_AddKeyFrame(typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& GlobalMapPoints);
std::vector<typeCovisibility> GRAPH_GetAllCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID);
std::vector<typeCovisibility> GRAPH_GetTopNCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID, const u64 N);
void GRAPH_UpdateCovisibility(typeCovisibilityGraph& CovisibilityGraph, const typeLocalMap& LocalMap, const std::vector<typePantoMapPoint>& GlobalMapPoints);

#endif // __GRAPH_PANTOGRAPH_HPP__

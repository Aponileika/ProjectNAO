#ifndef __GRAPH_PANTOGRAPH_HPP__
#define __GRAPH_PANTOGRAPH_HPP__
#include "CArenaAlloc.h"
#include <vector>
#include "KEY_Keyframe.hpp"
#include <map>
#include <algorithm>
#include "PT_Types.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef struct
{
    u64 KeyFrameID;
    u64 Covisibility;
}typeCovisibility;

typedef struct
{
    typePantoVector<std::unordered_map<u64, u64>> CovisibilityGraph;
    std::mutex Mutex;
}typeCovisibilityGraph;

void GRAPH_AddKeyFrame(typeCovisibilityGraph* CovisibilityGraph, const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints,
        const u64 ID);
typeCovisibility GRAPH_GetMostCovisibleFrame(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID);
std::vector<typeCovisibility> GRAPH_GetTopNCovisibleFrames( const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID, const u64 N);
std::vector<typeCovisibility> GRAPH_GetTopNExternalCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph,
        const std::vector<u64>& LocalKeyFrameIDs, const u64 N, const u64 ExcludedKeyFrameID);
void GRAPH_UpdateCovisibility( typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<typePantoMapPoint>& GlobalMapPoints, const u64 NewKeyFrameID,
        const std::vector<u64>& NewPointIDs);
void GRAPH_CullKeyFrame(typeCovisibilityGraph* CovisibilityGraph, u64 KeyFrameID);
void GRAPH_DecrementAll(typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<u64>& Nodes);
void GRAPH_DecrementAllOther(typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<u64>& Nodes, const u64 DecrementIndex);
void GRAPH_DecrementEdge( typeCovisibilityGraph* CovisibilityGraph,
        const u64 NodeA, const u64 NodeB);
void GRAPH_Log(const typeCovisibilityGraph& CovisibilityGraph);

#endif // __GRAPH_PANTOGRAPH_HPP__

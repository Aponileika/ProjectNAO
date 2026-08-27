#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include "PT_Types.hpp"
#include <KEY_KeyFrame.hpp>
#include <unordered_set>
#include "EP_CorrespondingPoints.hpp"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "GRAPH_PantoGraph.hpp"

typedef struct
{
    typePantoVector<typeKeyFrame> KeyFrames;
    typePantoVector<typePantoMapPoint> MapPoints;
}typeGlobalMap;

typedef struct
{
    std::vector<u64> KeyFrameIDs;
    std::vector<u64> MapPointIDs;
}typeLocalMapTracking;

typedef struct
{
    std::vector<u64> KeyFrameIDs;
    std::vector<u64> FixedKeyFrameIDs;
    std::vector<u64> MapPointIDs;
}typeLocalMap;

typedef struct
{
    fp64 TrackedRatio;
    fp64 MedianDepth;
}typeLocalMapInfo;

void MAP_AppendKeyFrame(typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame);
void MAP_InsertPreliminaryKeyFrame(typeGlobalMap& Map, typeKeyFrame& KeyFrame);
void MAP_RemovePreliminaryKeyFrame(typeGlobalMap& Map);
typeLocalMapTracking MAP_CreateLocalMapTracking(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame);
typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph);
typePantoVector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame);
typeLocalMapInfo MAP_MatchMapPointLocalMap(typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame);

void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap);
void MAP_CullRecentMapPoints(typePantoVector<u64>& RecentMapPointIndexes, typeGlobalMap& GlobalMap);
void MAP_CullObservationEdges(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap);

std::vector<u64> MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap,typeKeyFrame& NewKeyFrame, const typeCovisibilityGraph& CovisibilityGraph);
void MAP_LogGlobalMapPoses(const typeGlobalMap& GlobalMap);
void MAP_LogKeyFrameProjectionError(const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints);
void MAP_LogGlobalMapProjectionErrors(const typeGlobalMap& GlobalMap);
void MAP_RetriangulateLOST(typeGlobalMap& GlobalMap);

#endif // __MAP_MAPPING_HPP_

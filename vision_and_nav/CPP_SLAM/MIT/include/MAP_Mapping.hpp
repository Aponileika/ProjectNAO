#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include "PT_Types.hpp"
#include <KEY_KeyFrame.hpp>
#include <unordered_set>
#include "EP_CorrespondingPoints.hpp"
#include "Config.hpp"

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeGlobalMap;

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeLocalMap;

typedef struct
{
    fp64 TrackedRatio;
    fp64 MedianDepth;
}typeLocalMapInfo;

void MAP_AppendKeyFrame(typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame);
void MAP_InsertPreliminaryKeyFrame(typeGlobalMap& Map, typeKeyFrame& KeyFrame);
void MAP_RemovePreliminaryKeyFrame(typeGlobalMap& Map);
typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame);
std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame);
typeLocalMapInfo MAP_MatchMapPointLocalMap(const typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame);
void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap);
u64 MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame);
void MAP_LogGlobalMapPoses(const typeGlobalMap& GlobalMap);
void MAP_LogKeyFrameProjectionError(const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& GlobalMapPoints);
void MAP_LogGlobalMapProjectionErrors(const typeGlobalMap& GlobalMap);

#endif // __MAP_MAPPING_HPP_

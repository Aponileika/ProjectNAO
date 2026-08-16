#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include "PT_Types.hpp"
#include <KEY_Keyframe.hpp>
#include "EP_CorrespondingPoints.hpp"

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeGlobalMap;

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
}typeLocalMap;

void MAP_InsertPreliminaryKeyFrame(typeGlobalMap& Map, typeKeyFrame& KeyFrame);
void MAP_RemovePreliminaryKeyFrame(typeGlobalMap& Map);
typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame);
std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& NewKeyFrame);
fp64 MAP_MatchMapPointLocalMap(const typeGlobalMap& GlobalMap, const typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame);
void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap);
void MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap, typeKeyFrame NewKeyFrame);

#endif // __MAP_MAPPING_HPP_

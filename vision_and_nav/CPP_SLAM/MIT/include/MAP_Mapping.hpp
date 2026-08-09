#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include "PT_Types.hpp"
#include <KEY_Keyframe.hpp>

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeGlobalMap;

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
}typeLocalMap;

void MAP_InsertKeyFrame(typeGlobalMap& Map, const typeKeyFrame& KeyFrame);
typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame);
std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& NewKeyFrame);
fp64 MAP_SearchLocalMap(const typeGlobalMap& GlobalMap, const typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame);
void MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, const typeLocalMap& LocalMap);

#endif // __MAP_MAPPING_HPP_

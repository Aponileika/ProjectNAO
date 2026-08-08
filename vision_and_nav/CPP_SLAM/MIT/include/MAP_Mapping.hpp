#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include <KEY_Keyframe.hpp>
#include <PT_PantoMapPoints.hpp>

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

#endif // __MAP_MAPPING_HPP_

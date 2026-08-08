#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include <KEY_Keyframe.hpp>
#include <PT_PantoMapPoints.hpp>

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeMap;

void MAP_InsertKeyFrame(typeMap& Map, const typeKeyFrame KeyFrame);

#endif // __MAP_MAPPING_HPP_

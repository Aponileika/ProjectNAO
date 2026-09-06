#ifndef MAPPRIV_MAPPING_HPP_
#define MAPPRIV_MAPPING_HPP_
#include "MAP_Mapping.hpp"

typedef struct
{
    u64 RecentMapPointsCulled;

    u64 KeyFramesCulled;

    u64 MapPointsCulled;

    u64 ObservationEdgesCulled;
    u64 NumObservationEdgesPixelErrorHigh;
    u64 NumObservationEdgesFailedProjection;
    fp64 SumPixelErrorRemovedPixels;
    fp64 SquaredSumPixelErrorRemovedPixels;
}typeMappingData;

void MAPPriv_CullRecentMapPoint(typePantoMapPoint& MapPoint,
        u64 MapPointIndex,
        typeGlobalMap& GlobalMap,
        typeCovisibilityGraph& CovisibilityGraph);

#endif //  MAPPRIV_MAPPING_HPP_

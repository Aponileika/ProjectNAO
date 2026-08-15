#ifndef __PT_PANTOMAPPOINTS_HPP_
#define  __PT_PANTOMAPPOINTS_HPP_
#include "CArenaAlloc.h"
#include "Config.hpp"
#include <Eigen/Dense>
#include <PT_Types.hpp>

typePantoMapPoint PT_CreatePantoMapPoint(const Eigen::Vector4d& Point, const typeDescriptor& Descriptor, const std::pair<u64, u64>& KeyFrameIDs,
        const std::pair<u64, u64>& ImagePointIDs, const u64 ID);
void PT_AddObservation(typePantoMapPoint& MapPoint, const u64 KeyFrameID, const u64 ImagePointID);

#endif // __PT_PANTOMAPPOINTS_HPP_

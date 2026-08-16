#ifndef __PT_PANTOMAPPOINTS_HPP_
#define  __PT_PANTOMAPPOINTS_HPP_
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include <Eigen/Dense>
#include <PT_Types.hpp>

typePantoMapPoint PT_CreatePantoMapPoint(const Eigen::Vector4d& Point, const typeDescriptor& Descriptor, const std::pair<u64, u64>& KeyFrameIDs,
        const std::pair<u64, u64>& ImagePointIDs, const u64 ID);
void PT_AddObservation(typePantoMapPoint& MapPoint, const u64 KeyFrameID, const u64 ImagePointID);
bool PT_IsInfront(const Eigen::Vector4d& Point, const typeCamera& Camera);

#endif // __PT_PANTOMAPPOINTS_HPP_

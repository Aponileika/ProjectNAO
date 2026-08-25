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
bool PT_IsInfront(const Eigen::Vector4d& Point, const typeCamera& Camera);

inline fp64 PT_GetFoundRatio(const typePantoMapPoint& MapPoint)
{
    return static_cast<fp64>(MapPoint.NumFound) / static_cast<fp64>(MapPoint.NumVisible);
}

inline u64 PT_GetAge(const typePantoMapPoint& MapPoint, const u64 CurrentKFKID)
{
    return CurrentKFKID - MapPoint.FirstKFKID;
}

inline u64 PT_GetNumObservations(const typePantoMapPoint& MapPoint)
{
    return static_cast<u64>(MapPoint.KeyFrameIDs.size());
}

#endif // __PT_PANTOMAPPOINTS_HPP_

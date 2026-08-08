#ifndef __PT_PANTOMAPPOINTS_HPP_
#define  __PT_PANTOMAPPOINTS_HPP_
#include "CArenaAlloc.h"
#include "Config.hpp"
#include <Eigen/Dense>
#include <PT_Types.hpp>

void PT_CreatePoint(const Eigen::Vector4d& Point, std::vector<typeDescriptor> Descriptors);
void PT_AddObservation(typePantoMapPoint& MapPoint, const u64 KeyFrameID, const u64 ImagePointID);

#endif // __PT_PANTOMAPPOINTS_HPP_

#ifndef VIZPRIV_VISUALIZATION_HPP_
#define VIZPRIV_VISUALIZATION_HPP_
#include "VIZ_Visualization.hpp"
#include "KEY_Keyframe.hpp"
#include "MAP_Mapping.hpp"

void VIZPriv_WriteTrackingTrajectory(const std::vector<Eigen::Vector3d>& TrackingTrajectory, const std::string& SnapshotPath);
void VIZPriv_WriteCameras(const typePantoVector<typeKeyFrame>& KeyFrames, const std::string& SnapshotPath);
void VIZPriv_WriteImages(const typePantoVector<typeKeyFrame>& KeyFrames, const std::string& SnapshotPath);
void VIZPriv_WritePoints(const typeGlobalMap& GlobalMap, const std::string& SnapshotPath);
void VIZPriv_PublishSnapshot(const u64& SnapshotID);
void VIZPriv_LoadKeyFrameImages(const typePantoVector<typeKeyFrame>& KeyFrames);

#endif //  VIZPRIV_VISUALIZATION_HPP_

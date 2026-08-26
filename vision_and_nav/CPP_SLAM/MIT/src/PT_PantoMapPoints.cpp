#include "../include/PT_PantoMapPoints.hpp"

typePantoMapPoint PT_CreatePantoMapPoint(const Eigen::Vector4d& Point, const typeDescriptor& Descriptor, const std::pair<u64, u64>& KeyFrameIDs,
        const std::pair<u64, u64>& ImagePointIDs, const u64 ID)
{
    typePantoVector<u64> VecKeyFrameIDs(2);
    VecKeyFrameIDs[0] = KeyFrameIDs.first;
    VecKeyFrameIDs[1] = KeyFrameIDs.second;

    typePantoVector<u64> VecImagePointIDs(2);
    VecImagePointIDs[0] = ImagePointIDs.first;
    VecImagePointIDs[1] = ImagePointIDs.second;

    typePantoMapPoint MapPoint = 
    {
        .Point = PROJ_NormalizeToSpherical(Point),
        .Descriptor = Descriptor,
        .KeyFrameIDs = VecKeyFrameIDs,
        .ImagePointIDs = VecImagePointIDs,
        .ID = ID,
        .NumVisible = 1,
        .NumFound = 1,
        .FirstKFKID = KeyFrameIDs.second,
    };

    return MapPoint;
}

bool PT_IsInfront(const Eigen::Vector4d& Point, const typeCamera& Camera)
{
    const Eigen::Matrix3d& R =Camera.Pose.R;
    const Eigen::Vector3d& t =Camera.Pose.t;
    Eigen::Vector3d PointCart = PROJ_Homog2Cart(Point);

    PointCart = R * PointCart + t;
    return(PointCart.z() > 0.0f);
}



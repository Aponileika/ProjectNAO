#include "KEY_Keyframe.hpp"
#include "KEY_KeyFramePriv.hpp"

typeKeyFrame KEY_GetKeyFrame(const typeCamera& PredictedPose, const std::vector<typePantoMapPoint>& LastFrameMapPoints)
{
    cv::Mat Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame);
    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePoints(Descriptors.Points, Descriptors.Descriptors, LastFrameMapPoints, PredictedPose);
    typeKeyFrame KeyFrame = {
        .Points = ImagePoints,
        .Pose = PredictedPose,
        .ID = PANTO_ID_NOT_SET
    };
    return KeyFrame;
}

bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information)
{
    // TODO fuzzy inference
    return true;
}

void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, const u64& ID)
{
    KeyFrame.ID = ID;
}

std::vector<typePantoMapPoint> KEY_GetNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, u64 LatestMapPointID)
{
    const Eigen::Matrix3d EssentialMatrix21 = EP_GetEssentialMatrix21(KeyFrame1.Pose.Pose, KeyFrame2.Pose.Pose);
    std::vector<u64> Correspondences = EP_GetCorrespondences(KeyFrame1.Points, KeyFrame2.Points, EssentialMatrix21);
}

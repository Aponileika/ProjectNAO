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

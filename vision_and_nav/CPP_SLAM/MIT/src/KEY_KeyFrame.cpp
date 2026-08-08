#include "KEY_Keyframe.hpp"
#include "KEY_KeyFramePriv.hpp"

typeKeyFrame KEY_GetKeyFrame(Camera PredictedPose, std::vector<typePantoMapPoint> LastFrameMapPoints)
{
    cv::Mat Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame);
    std::vector<typePantoImagePoint> ImagePoints = PT_CreatePantoImagePoints(Descriptors.Points, Descriptors.Descriptors, LastFrameMapPoints);
    typeKeyFrame KeyFrame = {
        .Points = ImagePoints,
        .Pose = PredictedPose,
        .MapPointIDs = MapPointIDs
    };
    return KeyFrame;
}

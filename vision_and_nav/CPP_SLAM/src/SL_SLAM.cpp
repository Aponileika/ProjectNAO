#include "SL_SLAM.hpp"
#include "CM_Camera.hpp"
#include "VW_Views.hpp"

struct SLAM slam;

void SL_InitSlam()
{
    slam.Tview = VW_InitViewSet();
    slam.Tpoints = PT_InitPoints();
    slam.Tobs = OB_InitObs();
    slam.frame_pair = {};
    slam.CorrespExtractor = EP_CorrespExtract;
    FR_InitFrameGetter();
}

cv::Mat SL_GetNextFrame()
{
    int frameskip = 30;
    for(int i = 0; i < frameskip; i++)FR_GetFrame();
    return FR_GetFrame();
}

void SL_SlamLoop()
{
    slam.frame_pair.first = FR_GetFrame();
    slam.frame_pair.second = SL_GetNextFrame();
    PointPair2D corrp = slam.CorrespExtractor(slam.frame_pair.first, slam.frame_pair.second);   

    cv::Matx33d K = CM_GetIntrinsics()->K;
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(corrp.first, corrp.second, K,
            RANSACMETHOD, PROBECORRECT, RANSACEPIXELT, RANSACMAXITERS, mask);

    cv::Mat R, t;
    cv::recoverPose(E, corrp.first, corrp.second, K, R, t, mask);
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    VW_AddView(slam.Tview, CM_CreateCam(Rt));

    cv::Mat P1, P2;
    P1 = K * cv::Mat::eye(3, 4, CV_64F);
    P2 = K * Rt;

    cv::Mat4d points3d;

    cv::triangulatePoints(P1, P2, corrp.first, corrp.second, points3d);
    PT_AddPoints(slam.Tpoints, points3d);
}

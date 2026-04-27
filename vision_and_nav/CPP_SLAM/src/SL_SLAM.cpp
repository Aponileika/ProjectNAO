#include "../include/SL_SLAM.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "FR_Frames.hpp"
#include "OB_Observations.hpp"
#include "OP_BA.hpp"

struct SLAM slam;

static cv::Mat __SL_GetNextFrame();
static void __SL_SlamStart();

void SL_InitSlam()
{
    slam.Tview = VW_InitViewSet();
    slam.Tpoints = PT_InitPoints();
    slam.Tobs = OB_InitObs();
    slam.frame_pair = {};
    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}


static cv::Mat __SL_GetNextFrame()
{
    int frameskip = 90;
    for(int i = 0; i < frameskip; i++)FR_GetFrame();
    return FR_GetFrame();
}

void __SL_PrintSlam()
{
    LG_Log("Printing views\n");
    VW_Print(slam.Tview);
    LG_Log("Printing observations\n");
    OB_Print(slam.Tobs);
    LG_Log("Printing points\n");
    PT_Print(slam.Tpoints);
}

static void __SL_SlamStart()
{
    LG_Log("Getting first frame\n");
    slam.frame_pair.first = __SL_GetNextFrame();
    LG_Log("Getting second frame\n");
    slam.frame_pair.second = __SL_GetNextFrame();
    LG_Log("Getting corresponding points between frames\n");
    PointPair2D corrp = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   

    LG_Log("Getting intrinsics of camera\n");
    cv::Matx33d K = CM_GetIntrinsics()->K;
    cv::Mat mask;
    LG_Log("Finding essential matrix\n");
    LG_Log("Num points before RANSAC = %lld\n", corrp.first.size());
    cv::Mat E = cv::findEssentialMat(corrp.first, corrp.second, K,
            RANSACMETHOD, PROBECORRECT, RANSACEPIXELT, RANSACMAXITERS, mask);

    cv::Mat R, t;
    LG_Log("Recovering pose\n");
    int ninliers = cv::countNonZero(mask);
    cv::recoverPose(E, corrp.first, corrp.second, K, R, t, mask);
    int ninliersafterrecover = cv::countNonZero(mask);
    LG_Log("ninliers b4 recover = %d, after = %d\n", ninliers, ninliersafterrecover);
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    LG_Log("Adding first view\n");
    VW_AddView(slam.Tview, CM_CreateCam(cv::Mat::eye(3, 3, CV_64F), cv::Mat::eye(3, 1, CV_64F)));
    LG_Log("Adding second view\n");
    VW_AddView(slam.Tview, CM_CreateCam(R, t));

    cv::Mat P1, P2;
    P1 = K * cv::Mat::eye(3, 4, CV_64F);
    P2 = K * Rt;

    PointPair2D filteredcorrp = EP_FilterPointPairByMask(corrp, mask);
    corrp = std::move(filteredcorrp);
    LG_Log("number of corrp after masking = %lld\n", corrp.first.size());

    cv::Mat points3d;
    LG_Log("Triangulating\n");
    cv::triangulatePoints(P1, P2, corrp.first, corrp.second, points3d);

    LG_Log("Adding points\n");
    PT_AddPoints(slam.Tpoints, points3d);
    LG_Log("Adding observations\n");
    OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, corrp);
    __SL_PrintSlam();
}


void __SL_SlamLoop()
{
    OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
    LG_Log("Printing views after BA\n");
    VW_Print(slam.Tview);
    slam.frame_pair.first = slam.frame_pair.second;
    LG_Log("Getting new frame in SLAM loop\n");
    slam.frame_pair.second = __SL_GetNextFrame();
    LG_Log("Getting corresponding points in SLAM loop\n");
    PointPair2D corrp = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   
    LG_Log("Solving pnp\n");
    OB_SolvePnP(corrp, slam.Tview, slam.Tobs, slam.Tpoints);
    LG_Log("Pre BA\n");
    VW_Print(slam.Tview);
    OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
    LG_Log("After BA\n");
    VW_Print(slam.Tview);
    //TODO triangulate new points using epipolar constraint, then continue looping
    //After that TODO = ALOT of improvements
}

void SL_SlamLoop()
{
    LG_Log(SLAMSTARTMSG);
    __SL_SlamStart();
    __SL_SlamLoop();
    return;
}

#include "../include/SL_SLAM.hpp"
#include "CM_Camera.hpp"
#include "EP_CorrespondingPoints.hpp"
#include "FEAT_Features.hpp"
#include "FR_Frames.hpp"
#include "OB_Observations.hpp"
#include "OP_BA.hpp"
#include "VIZ_Visualization.hpp"
#include "VW_Views.hpp"
#include <chrono>
#include <utility>

struct SLAM slam;

static cv::Mat __SL_GetNextFrame(u64 num_views);
void __SL_PrintSlam();
static void __SL_SlamStart();
void __SL_SlamLoopBundle();
PointPair2D __SL_SlamLoopPnP();

void SL_InitSlam()
{
    slam.Tview = VW_InitViewSet();
    slam.Tpoints = PT_InitPoints();
    slam.Tobs = OB_InitObs();
    slam.frame_pair = {};
    void* extr = static_cast<void*>(AKAZE_InitAKAZE(AKAZEthreshold));
    LG_Log("[SL_InitSlam] AKAZE init ptr = %p\n", extr);
    EP_InitCPointExtractor(extr, AKAZE_GetMatches);

    // void* extr = static_cast<void*>(ORB_InitORB());
    // EP_InitCPointExtractor(extr, ORB_GetMatches);
    FR_InitFrameGetter();
}

static cv::Mat __SL_GetNextFrame(u64 num_views)
{
    cv::Mat frame = FR_GetFrame(num_views);
    return frame;
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
    const cv::Matx33d K = CM_GetIntrinsics()->K;
    LG_Log("Getting first frame\n");
    slam.frame_pair.first = __SL_GetNextFrame(slam.Tview->views.size());
    LG_Log("Getting second frame\n");
    bool isinit = false;
    while(!isinit)
    {
        slam.frame_pair.second = __SL_GetNextFrame(slam.Tview->views.size());
        LG_Log("[__SL_SlamStart] Getting corresponding points between frames\n");
        PointPair2D corrp = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   
        // EP_DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, corrp.first, corrp.second);

        LG_Log("Getting intrinsics of camera\n");
        LG_Log("Finding essential matrix\n");
        LG_Log("Num points before RANSAC = %lld\n", corrp.first.size());
        if(corrp.first.size() < 5)continue;
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(corrp.first, corrp.second, K, 
                    RANSACMETHOD, PROBECORRECT, RANSACEPIXELT, RANSACMAXITERS, mask);
                
        cv::Mat R, t;
        cv::Mat poseMask = mask.clone();
        
        LG_Log("Recovering pose\n");
        int ninliersafterrecover = cv::recoverPose(E, corrp.first, corrp.second, K, R, t, poseMask);

        int ninliers = cv::countNonZero(mask);
        LG_Log("ninliers = %d\n", ninliersafterrecover);
        if(ninliersafterrecover < InitFrameThresholdCorrp)
        {
            LG_Log("ninliers too few after recover %d, need %d\n", ninliersafterrecover,
                    InitFrameThresholdCorrp);
            continue;
        }
        LG_Log("ninliers b4 recover = %d, after = %d\n", ninliers, ninliersafterrecover);
        cv::Mat Rt;
        cv::hconcat(R, t, Rt);
        LG_Log("Adding first view\n");
        VW_AddView(slam.Tview, CM_CreateCam(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), 0));
        LG_Log("Adding second view\n");
        VW_AddView(slam.Tview, CM_CreateCam(R, t, 1));

        cv::Mat P1, P2;
        P1 = K * cv::Mat::eye(3, 4, CV_64F);
        P2 = K * Rt;

        PointPair2D filteredcorrp = EP_FilterPointPairByMask(corrp, poseMask);
        corrp = std::move(filteredcorrp);
        LG_Log("number of corrp after masking = %lld\n", corrp.first.size());
        //EP_DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, corrp.first, corrp.second);

        cv::Mat points3d;
        LG_Log("Triangulating\n");
        cv::triangulatePoints(P1, P2, corrp.first, corrp.second, points3d);

        LG_Log("Adding points\n");
        PT_AddPoints(slam.Tpoints, points3d);
        LG_Log("Adding observations\n");
        OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, corrp);
        isinit = true;
    }
    __SL_PrintSlam();
}

void __SL_SlamLoopBundle()
{
    OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
}

PointPair2D __SL_SlamLoopPnP()
{
    PointPair2D corrp = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   
    LG_Log("[__SL_SlamLoopPnP] Found %d correspondences betwen new frame pair\n", corrp.first.size());
    if(corrp.first.size() < NewFrameCorrpThreshold)
    {
        LG_Log("not enough points found for new frame found %d, need %d\n", 
                corrp.first.size(), NewFrameCorrpThreshold);
        return {};
    }
    LG_Log("Solving pnp\n");
    struct PnPret pnpret = OB_SolvePnP(corrp, slam.Tview, slam.Tobs, slam.Tpoints);
    if(pnpret.ret == PNP_NOT_ENOUGH_2D3D)
    {
        LG_Log("not enough 2d3d in pnp\n");
        return {};
    }
    if(pnpret.ret == PNP_NOT_ENOUGH_NONPNP)
    {
        LG_Log("not enough non 2d3d in pnp\n");
        return {};
    }
    return pnpret.nonpnpPoints;
}

void __SL_SlamLoop()
{
    const CameraIntrinsics* ci = CM_GetIntrinsics();
    const cv::Matx33d K = ci->K;
    bool added_view = true;
    for(int i = 0; i < 1; i++)
    {
        LG_Log("Starting SLAM loop %d\n", i);
        if(added_view)
        {
            OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
            LG_Log("Printing views after BA\n");
            VW_Print(slam.Tview);
            slam.frame_pair.first = slam.frame_pair.second;
        }
        LG_Log("Getting new frame in SLAM loop\n");
        slam.frame_pair.second = __SL_GetNextFrame(slam.Tview->views.size());
        LG_Log("Getting corresponding points in SLAM loop\n");
        PointPair2D nonpnpcorrp = __SL_SlamLoopPnP();
        if(nonpnpcorrp.first.size() == 0)
        {
            LG_Log("Pnp block failed, getting new frame\n");
            added_view = false;
        }
        added_view = true;
        LG_Log("Getting cam\n");
        struct Camera cam2 = slam.Tview->views[slam.Tview->last_sz];
        LG_Log("Getting E from cam\n");
        struct Camera* two_latest = VW_GetTwoLatestCams(slam.Tview);
        cv::Mat R1 = two_latest[0].R;
        cv::Mat t1 = two_latest[0].t;

        cv::Mat R2 = two_latest[1].R;
        cv::Mat t2 = two_latest[1].t;
        std::pair<cv::Mat, cv::Mat> R21t21 = EP_GetR21t21(R1, t1, R2, t2);
        cv::Mat E = EP_EFromRigid(R21t21.first, R21t21.second);
        // EP_DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, nonpnpcorrp.first, nonpnpcorrp.second);
        LG_Log("Finding corrp with epipolar constraint\n");

        PointPair2D corr_p = EP_FindCorrpEpipolar(nonpnpcorrp, E);
        LG_Log("Found %lld corresponding points after filtering\n", corr_p.first.size());
        if(corr_p.first.size() < 1)
        {
            LG_Log("Found no corresponding points after filtering\n");
            continue;
        }

        struct Camera cam1 = slam.Tview->views[slam.Tview->last_sz - 1];
        cv::Mat Rt1st;
        cv::Mat Rt2nd;
        cv::hconcat(cam1.R, cam1.t, Rt1st);
        cv::hconcat(cam2.R, cam2.t, Rt2nd);

        cv::Mat P1, P2;
        P1 = K * Rt1st;
        P2 = K * Rt2nd;

        cv::Mat points3d;
        LG_Log("Triangulating new points\n");
        cv::triangulatePoints(P1, P2, corr_p.first, corr_p.second, points3d);
        LG_Log("Adding points\n");
        PT_AddPoints(slam.Tpoints, points3d);
        LG_Log("Adding observations\n");
        OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, corr_p);
        __SL_PrintSlam();
    }
    const std::string path = "./colmap/sparse/0/";
    VIZ_WriteColmap(*(slam.Tobs), *(slam.Tpoints), *(slam.Tview), path);
}

void SL_SlamLoop()
{
    LG_Log(SLAMSTARTMSG);
    __SL_SlamStart();
    __SL_SlamLoop();
    return;
}

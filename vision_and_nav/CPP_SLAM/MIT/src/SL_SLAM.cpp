#include "../include/SL_SLAM.hpp"
#include "CM_Camera.hpp"
#include "EP_CorrespondingPoints.hpp"
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
    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

static cv::Mat __SL_GetNextFrame(u64 num_views)
{
    cv::Mat frame = FR_GetFrame(num_views);
    return frame;
}

void __SL_PrintSlam()
{
    LG_Log(LogSeverity::DBG, "Printing views\n");
    VW_Print(slam.Tview);
    LG_Log(LogSeverity::DBG, "Printing observations\n");
    OB_Print(slam.Tobs);
    LG_Log(LogSeverity::DBG, "Printing points\n");
    PT_Print(slam.Tpoints);
}

static void __SL_SlamStart()
{
    const cv::Matx33d K = CM_GetIntrinsics()->K;
    LG_Log(LogSeverity::DBG, "Getting first frame\n");
    slam.frame_pair.first = __SL_GetNextFrame(slam.Tview->views.size());
    LG_Log(LogSeverity::DBG, "Getting second frame\n");
    bool isinit = false;
    while(!isinit)
    {
        slam.frame_pair.second = __SL_GetNextFrame(slam.Tview->views.size());
        LG_Log(LogSeverity::DBG, "[__SL_SlamStart] Getting corresponding points between frames\n");
        MatchesRet matches = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   
        const auto& corrp = matches.Matches; 

        LG_Log(LogSeverity::DBG, "[__SL_SlamStart]Finding essential matrix\n");
        LG_Log(LogSeverity::DBG, "[__SL_SlamStart]Num points before RANSAC = %lld\n", corrp.first.size());
        if(corrp.first.size() < 5)continue;
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(corrp.first, corrp.second, K, 
                    OPENCV_RANSACMETHOD, OPENCV_PROBECORRECT, OPENCV_RANSACEPIXELT, OPENCV_RANSACMAXITERS, mask);
                
        cv::Mat R, t;
        cv::Mat poseMask = mask.clone();
        
        LG_Log(LogSeverity::DBG, "Recovering pose\n");
        int ninliersafterrecover = cv::recoverPose(E, corrp.first, corrp.second, K, R, t, poseMask);

        int ninliers = cv::countNonZero(mask);
        LG_Log(LogSeverity::DBG, "ninliers = %d\n", ninliersafterrecover);
        if(ninliersafterrecover < PANTO_INITFRAMETHRESHOLDCORRP)
        {
            LG_Log(LogSeverity::DBG, "ninliers too few after recover %d, need %d\n", ninliersafterrecover,
                    PANTO_INITFRAMETHRESHOLDCORRP);
            continue;
        }
        LG_Log(LogSeverity::DBG, "ninliers b4 recover = %d, after = %d\n", ninliers, ninliersafterrecover);
        cv::Mat Rt;
        cv::hconcat(R, t, Rt);
        LG_Log(LogSeverity::DBG, "Adding first view\n");
        VW_AddView(slam.Tview, CM_CreateCam(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), 0));
        LG_Log(LogSeverity::DBG, "Adding second view\n");
        VW_AddView(slam.Tview, CM_CreateCam(R, t, 1));

        cv::Mat P1, P2;
        P1 = K * cv::Mat::eye(3, 4, CV_64F);

        PointPair2D FilteredCorrespondences = EP_FilterPointPairByMask(corrp, poseMask);
        LG_Log(LogSeverity::DBG, "number of corrp after masking = %lld\n", corrp.first.size());
        EP_DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, FilteredCorrespondences.first, FilteredCorrespondences.second);

        LG_Log(LogSeverity::DBG, "Triangulating\n");
        LG_Log(LogSeverity::DBG, "[__SL_SlamStart] first point = (%lf, %lf)\n", FilteredCorrespondences.first[0].x, FilteredCorrespondences.first[0].y);
        const std::vector<std::vector<Eigen::Vector3d>> PixelCoords = PANTO_PointPair2Eigen(FilteredCorrespondences);
        Eigen::Matrix<fp64, 3, 4> T1 = Eigen::Matrix<fp64, 3, 4>::Identity();
        Eigen::MatrixXd T2 = PANTO_Cv2Eigen<Eigen::MatrixXd>(Rt);
        u64 n = PixelCoords.size();
        std::vector<std::vector<Eigen::Matrix4d>> T(n);
        for(u64 i = 0; i < n; i++)
        {
            T[i].resize(2);
            T[i][0] = T1;
            T[i][1] = T2;
        }
        
        Eigen::Matrix3d K = PANTO_Cv2Eigen<Eigen::Matrix3d>(CM_GetIntrinsics()->K);
        std::vector<Eigen::Vector4d> Points3D = PROJ_TriangulateLOST(PixelCoords, T, K);
        LG_Log(LogSeverity::DBG, "Adding points\n");
        PT_AddPoints(slam.Tpoints, Points3D);
        LG_Log(LogSeverity::DBG, "Adding observations\n");
        OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, FilteredCorrespondences);
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
    DescRet CorrespondingPoints = EP_GetDescriptors(slam.frame_pair.second);   
    LG_Log(LogSeverity::DBG, "Solving pnp\n");
    struct PnPret pnpret = OB_SolvePnP(CorrespondingPoints.Matches, slam.Tview, slam.Tobs, slam.Tpoints);
    if(pnpret.ret == PNP_NOT_ENOUGH_2D3D)
    {
        LG_Log(LogSeverity::DBG, "not enough 2d3d in pnp\n");
        return {};
    }
    if(pnpret.ret == PNP_NOT_ENOUGH_NONPNP)
    {
        LG_Log(LogSeverity::DBG, "not enough non 2d3d in pnp\n");
        return {};
    }
    return pnpret.nonpnpPoints;
}

void __SL_SlamLoop(i32 num_loops)
{
    const CameraIntrinsics* ci = CM_GetIntrinsics();
    const cv::Matx33d K = ci->K;
    bool added_view = true;
    for(int i = 0; i < num_loops; i++)
    {
        LG_Log(LogSeverity::DBG, "Starting SLAM loop %d\n", i);
        if(added_view)
        {
            OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
            LG_Log(LogSeverity::DBG, "Printing views after BA\n");
            VW_Print(slam.Tview);
            slam.frame_pair.first = slam.frame_pair.second;
        }
        LG_Log(LogSeverity::DBG, "Getting new frame in SLAM loop\n");
        slam.frame_pair.second = __SL_GetNextFrame(slam.Tview->views.size());
        if(slam.frame_pair.second.empty())break;
        LG_Log(LogSeverity::DBG, "Getting corresponding points in SLAM loop\n");
        PointPair2D nonpnpcorrp = __SL_SlamLoopPnP();
        if(nonpnpcorrp.first.size() == 0)
        {
            LG_Log(LogSeverity::DBG, "Pnp block failed, getting new frame\n");
            added_view = false;
        }
        added_view = true;
        LG_Log(LogSeverity::DBG, "Getting cam\n");
        struct Camera cam2 = slam.Tview->views[slam.Tview->last_sz];
        LG_Log(LogSeverity::DBG, "Getting E from cam\n");
        struct Camera* two_latest = VW_GetTwoLatestCams(slam.Tview);
        cv::Mat R1 = two_latest[0].R;
        cv::Mat t1 = two_latest[0].t;

        cv::Mat R2 = two_latest[1].R;
        cv::Mat t2 = two_latest[1].t;
        std::pair<cv::Mat, cv::Mat> R21t21 = EP_GetR21t21(R1, t1, R2, t2);
        cv::Mat E = EP_EFromRigid(R21t21.first, R21t21.second);
        LG_Log(LogSeverity::DBG, "Finding corrp with epipolar constraint\n");

        PointPair2D corrPoints = EP_FindCorrpEpipolar(nonpnpcorrp, E);
        // EP_DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, corr_p.first, corr_p.second);
        LG_Log(LogSeverity::DBG, "Found %lld corresponding points after filtering\n", corrPoints.first.size());
        if(corrPoints.first.size() < 1)
        {
            LG_Log(LogSeverity::DBG, "Found no corresponding points after filtering\n");
            continue;
        }

        struct Camera cam1 = slam.Tview->views[slam.Tview->last_sz - 1];
        cv::Mat Rt1st;
        cv::Mat Rt2nd;
        cv::hconcat(cam1.R, cam1.t, Rt1st);
        cv::hconcat(cam2.R, cam2.t, Rt2nd);

        cv::Mat P1, P2;

        const std::vector<std::vector<Eigen::Vector3d>> pixelCoords = PANTO_PointPair2Eigen(corrPoints);
        Eigen::MatrixXd T1 = PANTO_Cv2Eigen<Eigen::MatrixXd>(Rt1st);
        Eigen::MatrixXd T2 = PANTO_Cv2Eigen<Eigen::MatrixXd>(Rt2nd);
        u64 n = pixelCoords.size();
        std::vector<std::vector<Eigen::Matrix4d>> T(n);
        for(u64 i = 0; i < n; i++)
        {
            T[i].resize(2);
            T[i][0] = T1;
            T[i][1] = T2;
        }
        
        Eigen::Matrix3d K = PANTO_Cv2Eigen<Eigen::Matrix3d>(CM_GetIntrinsics()->K);
        std::vector<Eigen::Vector4d> points3d = PROJ_TriangulateLOST(pixelCoords, T, K);

        LG_Log(LogSeverity::DBG, "Triangulating new points\n");
        LG_Log(LogSeverity::DBG, "Adding points\n");
        PT_AddPoints(slam.Tpoints, points3d);
        LG_Log(LogSeverity::DBG, "Adding observations\n");
        OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, corrPoints);
        __SL_PrintSlam();
    }
    OP_BundleAdjust(slam.Tview, slam.Tobs, slam.Tpoints);
    const std::string path = "./colmap/sparse/0/";
    VIZ_WriteColmap(*(slam.Tobs), *(slam.Tpoints), *(slam.Tview), path);
}

void SL_SlamLoop(i32 num_loops)
{
    LG_Log(LogSeverity::DBG, PANTO_SLAMSTARTMSG);
    __SL_SlamStart();
    __SL_SlamLoop(num_loops);
    return;
}

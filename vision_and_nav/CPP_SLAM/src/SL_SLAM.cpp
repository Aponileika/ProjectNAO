#include "../include/SL_SLAM.hpp"
#include "include/EP_CorrespondingPoints.hpp"

struct SLAM slam;
static void DrawCorrespondences
    (const cv::Mat& img1,
    const cv::Mat& img2,
    const std::vector<cv::Point2d>& pts1,
    const std::vector<cv::Point2d>& pts2,
    const cv::Mat& mask);
void SL_InitSlam()
{
    slam.Tview = VW_InitViewSet();
    slam.Tpoints = PT_InitPoints();
    slam.Tobs = OB_InitObs();
    slam.frame_pair = {};
    EP_InitCPointExtractor();
    FR_InitFrameGetter();
}

static void DrawCorrespondences
(
    const cv::Mat& img1,
    const cv::Mat& img2,
    const std::vector<cv::Point2d>& pts1,
    const std::vector<cv::Point2d>& pts2,
    const cv::Mat& mask)
{
    const std::string windowName = "Correspondences";
    CV_Assert(pts1.size() == pts2.size());

    cv::Mat left, right;
    if (img1.channels() == 1) cv::cvtColor(img1, left, cv::COLOR_GRAY2BGR);
    else left = img1.clone();

    if (img2.channels() == 1) cv::cvtColor(img2, right, cv::COLOR_GRAY2BGR);
    else right = img2.clone();

    int rows = std::max(left.rows, right.rows);
    int cols = left.cols + right.cols;

    cv::Mat canvas(rows, cols, CV_8UC3, cv::Scalar(0,0,0));
    left.copyTo(canvas(cv::Rect(0, 0, left.cols, left.rows)));
    right.copyTo(canvas(cv::Rect(left.cols, 0, right.cols, right.rows)));

    for (size_t i = 0; i < pts1.size(); ++i)
    {
        bool usePoint = true;
        if (!mask.empty())
        {
            // mask is usually Nx1 uchar
            usePoint = mask.at<uchar>((int)i) != 0;
        }

        if (!usePoint) continue;

        cv::Point2f p1 = pts1[i];
        cv::Point2f p2 = pts2[i];
        p2.x += static_cast<float>(left.cols); // shift right image points

        cv::Scalar color(
            (i * 53) % 255,
            (i * 97) % 255,
            (i * 193) % 255
        );

        cv::circle(canvas, p1, 4, color, 2);
        cv::circle(canvas, p2, 4, color, 2);
        cv::line(canvas, p1, p2, color, 1);
    }

    cv::imshow(windowName, canvas);
    cv::waitKey(0);
}

cv::Mat SL_GetNextFrame()
{
    int frameskip = 90;
    for(int i = 0; i < frameskip; i++)FR_GetFrame();
    return FR_GetFrame();
}

void SL_SlamLoop()
{
    printf("Getting first frame\n");
    slam.frame_pair.first = SL_GetNextFrame();
    printf("Getting second frame\n");
    slam.frame_pair.second = SL_GetNextFrame();
    printf("Getting corresponding points between frames\n");
    PointPair2D corrp = EP_CorrespExtract(slam.frame_pair.first, slam.frame_pair.second);   

    printf("Getting intrinsics of camera\n");
    cv::Matx33d K = CM_GetIntrinsics()->K;
    cv::Mat mask;
    printf("Finding essential matrix\n");
    cv::Mat E = cv::findEssentialMat(corrp.first, corrp.second, K,
            RANSACMETHOD, PROBECORRECT, RANSACEPIXELT, RANSACMAXITERS, mask);
    int ninliers = cv::countNonZero(mask);

    cv::Mat R, t;
    printf("Recovering pose\n");
    cv::recoverPose(E, corrp.first, corrp.second, K, R, t, mask);
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    printf("Adding first view\n");
    VW_AddView(slam.Tview, CM_CreateCam(cv::Mat::eye(3, 4, CV_64F)));
    printf("Adding second view\n");
    VW_AddView(slam.Tview, CM_CreateCam(Rt));

    cv::Mat P1, P2;
    P1 = K * cv::Mat::eye(3, 4, CV_64F);
    P2 = K * Rt;

    cv::Mat points3d;
    printf("Triangulating\n");
    cv::triangulatePoints(P1, P2, corrp.first, corrp.second, points3d);

    printf("Adding points\n");
    PT_AddPoints(slam.Tpoints, points3d);
    printf("Adding observations\n");
    OB_AddObs(slam.Tobs, slam.Tview, slam.Tpoints, corrp);

    printf("Printing views\n");
    VW_Print(slam.Tview);
    printf("Printing observations\n");
    OB_Print(slam.Tobs);
    printf("Printing points\n");
    PT_Print(slam.Tpoints);
    std::cout << "Inliers after RANSAC: " << ninliers << "\n";
    DrawCorrespondences(slam.frame_pair.first, slam.frame_pair.second, corrp.first, corrp.second, mask);
    return;
}

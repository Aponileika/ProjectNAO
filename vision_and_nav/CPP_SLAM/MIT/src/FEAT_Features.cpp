#include "FEAT_Features.hpp"
#include "LG_Logging.hpp"

/*
 *************************************************************
 *************************************************************
 *************************************************************
 *                          ORB
 *************************************************************
 *************************************************************
 *************************************************************
 * */

struct OrbExtractor* ORB_InitORB()
{
    struct OrbExtractor* orb = new struct OrbExtractor{};
    orb->orb = ORB_SLAM::ORBextractor();
    orb->matcher = cv::BFMatcher(cv::NORM_HAMMING, false);
    orb->matchratio = MATCHRATIO;
    return orb;
}

void ORB_Destroy(struct OrbExtractor* orb)
{
    delete(orb);
}

PointPair2D ORB_GetMatches(void* extractor, cv::Mat img1, cv::Mat img2)
{
    LG_Log("[ORB_GetMatches] extr void* = %p\n", extractor);
    struct OrbExtractor* orb = static_cast<struct OrbExtractor*>(extractor);
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    orb->orb(img1, {}, kp1, des1);
    orb->orb(img2, {}, kp2, des2);
    struct CameraIntrinsics* ci = CM_GetIntrinsics();
    cv::Matx33d K = ci->K;
    cv::Vec<fp64, 5> distcoeffs = ci->distcoeffs;

    if(des1.empty() || des2.empty())
    {
        std::cerr << "No descriptors found inf EP_CorrespExtract\n";
        return {};
    }

    std::vector<std::vector<cv::DMatch>> matches;
    orb->matcher.knnMatch(des1, des2, matches, 2);

    std::vector<cv::DMatch> good;
    for(const auto&m : matches)
    {
        if(m.size() == 2 && m[0].distance < orb->matchratio * m[1].distance)
        {
            good.push_back(m[0]);
        }
    }
    PointPair2D out;

    for (const auto& m : good) {
        out.first.push_back(kp1[m.queryIdx].pt);
        out.second.push_back(kp2[m.trainIdx].pt);
    }

    std::vector<cv::Point2d> p1d, p2d;
    cv::undistortPoints(out.first, p1d, K, distcoeffs, cv::noArray(), K);
    cv::undistortPoints(out.second, p2d, K, distcoeffs, cv::noArray(), K);

    out = PointPair2D(p1d, p2d);
    return out;
}

/*
 *************************************************************
 *************************************************************
 *************************************************************
 *                          AKAZE
 *************************************************************
 *************************************************************
 *************************************************************
 * */

struct OpenCVExtractAKAZE* AKAZE_InitAKAZE(fp64 threshold)
{
    LG_Log("[AKAZE_InitAKAZE] initing akaze with %lf\n", threshold);
    struct OpenCVExtractAKAZE* akaze = new struct OpenCVExtractAKAZE{};
    akaze->akaze = cv::AKAZE::create();
    akaze->akaze->setThreshold(threshold);
    akaze->akaze->setNOctaves(4);
    akaze->akaze->setNOctaveLayers(4);
    akaze->matcher = cv::BFMatcher(cv::NORM_HAMMING, false);
    akaze->matchratio = MATCHRATIO;
    return akaze;
}

void AKAZE_destroy(struct OpenCVExtractAKAZE* akaze)
{
    delete(akaze);
}

PointPair2D AKAZE_GetMatches(void* extractor, cv::Mat img1, cv::Mat img2)
{
    LG_Log("[AKAZE_GetMatches] extr void* = %p\n", extractor);
    struct OpenCVExtractAKAZE* akaze = static_cast<struct OpenCVExtractAKAZE*>(extractor);
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    LG_Log("[AKAZE_GetMatches] img1 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img1.empty(), img1.rows, img1.cols, img1.type(), img1.channels(), img1.data);

    LG_Log("[AKAZE_GetMatches] img2 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img2.empty(), img2.rows, img2.cols, img2.type(), img2.channels(), img2.data);
    LG_Log("[AKAZE_GetMatches] detect and compute img1\n");
    akaze->akaze->detectAndCompute(img1, cv::noArray(), kp1, des1);

    LG_Log("[AKAZE_GetMatches] detect and compute img2\n");
    akaze->akaze->detectAndCompute(img2, cv::noArray(), kp2, des2);
    struct CameraIntrinsics* ci = CM_GetIntrinsics();
    cv::Matx33d K = ci->K;
    cv::Vec<fp64, 5> distcoeffs = ci->distcoeffs;

    if(des1.empty() || des2.empty())
    {
        std::cerr << "No descriptors found inf EP_CorrespExtract\n";
        return {};
    }

    std::vector<std::vector<cv::DMatch>> matches;
    akaze->matcher.knnMatch(des1, des2, matches, 2);

    std::vector<cv::DMatch> good;
    for(const auto&m : matches)
    {
        if(m.size() == 2 && m[0].distance < akaze->matchratio * m[1].distance)
        {
            good.push_back(m[0]);
        }
    }
    PointPair2D out;

    for (const auto& m : good) {
        out.first.push_back(kp1[m.queryIdx].pt);
        out.second.push_back(kp2[m.trainIdx].pt);
    }

    std::vector<cv::Point2d> p1d, p2d;
    cv::undistortPoints(out.first, p1d, K, distcoeffs, cv::noArray(), K);
    cv::undistortPoints(out.second, p2d, K, distcoeffs, cv::noArray(), K);

    out = PointPair2D(p1d, p2d);
    return out;
}


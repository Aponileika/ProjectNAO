#include "../include/EP_CorrespondingPoints.hpp"

struct OrbExtractor orb;

void EP_InitCPointExtractor()
{
    orb.orb = cv::ORB::create(NFEATURES);
    orb.matcher = cv::BFMatcher(cv::NORM_HAMMING);
    orb.matchratio = MATCHRATIO;
}

PointPair2D EP_CorrespExtract(cv::Mat img1, cv::Mat img2)
{
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    orb.orb->detectAndCompute(img1, cv::noArray(), kp1, des1);
    orb.orb->detectAndCompute(img2, cv::noArray(), kp2, des2);

    if(des1.empty() || des2.empty())
    {
        std::cerr << "No descriptors found inf EP_CorrespExtract\n";
        return {};
    }

    std::vector<std::vector<cv::DMatch>> matches;
    orb.matcher.knnMatch(des1, des2, matches, 2);

    std::vector<cv::DMatch> good;
    for(const auto&m : matches)
    {
        if(m.size() == 2 && m[0].distance < orb.matchratio * m[1].distance)
        {
            good.push_back(m[0]);
        }
    }
    PointPair2D out;

    for (const auto& m : good) {
    out.first.push_back(kp1[m.queryIdx].pt);
    out.second.push_back(kp2[m.trainIdx].pt);
    }

    return out;
}


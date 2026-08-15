#include "../include/EP_CorrespondingPoints.hpp"
#include "EP_CorrespondingPointsPriv.hpp"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"
#include "PT_Points.hpp"

struct AKAZEExtract AkazeExtract;

void EP_InitCPointExtractor(void)
{
    __EP_InitAkaze();
}

MatchesRet EP_GetCorrespondences(cv::Mat Img1, cv::Mat Img2)
{
    LG_Log(LogSeverity::DBG, "[EP_GetCorrespondences] Getting Correspondences\n");
    MatchesRet Ret = __EP_GetMatches(Img1, Img2);
    return Ret;
}

DescRet EP_GetDescriptors(cv::Mat Img)
{
    LG_Log(LogSeverity::DBG, "[EP_GetDescriptors] Getting Descriptors\n");
    DescRet Ret = __EP_GetDesc(Img);
    return Ret;
}

MatchesRet EP_GetMatches(std::pair<cv::Mat, cv::Mat> Descriptors, std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> Points)
{
    LG_Log(LogSeverity::DBG, "[EP_GetMatches] Getting Matches\n");
    MatchesRet Ret = __EP_GetCorrespondingPoints(Descriptors, Points);
    return Ret;
}

cv::Mat EP_EFromRigid(cv::Mat R, cv::Mat t)
{
    //IREG p 156, (10.53)
    //This gives us E21
    return R.t() * __EP_CrossProdMat(t);
    //return __EP_CrossProdMat(t) * R;
}

std::pair<cv::Mat, cv::Mat> EP_GetR21t21(cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2)
{
    cv::Mat R21 = R2 * R1.t();
    cv::Mat t21 = t2 - R21 * t1;
    std::pair<cv::Mat, cv::Mat> R21t21(R21, t21);
    return R21t21;
}

Eigen::Matrix3d EP_GetFundamentalMatrix21(const typeCameraPose& Pose1, const typeCameraPose& Pose2)
{
    const Eigen::Matrix3d R21 = Pose2.R * Pose1.R.transpose();
    const Eigen::Vector3d t21 = Pose2.t - R21 * Pose1.t;
    const Eigen::Matrix2d& K = CM_GetIntrinsics()->K;
    const Eigen::Matrix2d& K_inv = K.inverse();
    const Eigen::Matrix2d& F21 = K_inv.transpose() * PROJ_CrossProductMatrix(t21) * R21 * K_inv;
    return F21;
}

bool EP_CheckEpipolarConstraint(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2,
    const Eigen::Matrix3d& F21, const Eigen::Matrix3d& F12)
{
    const Eigen::Vector3d HomogPoint1 = PROJ_Cart2Homog(Point1);
    const Eigen::Vector3d HomogPoint2 = PROJ_Cart2Homog(Point2);

    Eigen::Vector3d EpipolarLine1 = F21 * HomogPoint2;
    EpipolarLine1 /= sqrt(EpipolarLine1.x()*EpipolarLine1.x() + EpipolarLine1.y()*EpipolarLine1.y());
    fp64 Distance1 = HomogPoint1.transpose() * EpipolarLine1;

    Eigen::Vector3d EpipolarLine2 = F12 * HomogPoint1;
    EpipolarLine2 /= sqrt(EpipolarLine2.x()*EpipolarLine2.x() + EpipolarLine2.y()*EpipolarLine2.y());
    fp64 Distance2 = HomogPoint2.transpose() * EpipolarLine2;

    fp64 MeanDistance = (Distance1 + Distance2) * 0.5f;
    return MeanDistance < PANTO_EPIPOLARTRESHOLD;
}

PointPair2D EP_FindCorrpEpipolar(const PointPair2D& corrp, const cv::Mat& E)
{
    PointPair2D corr_p;
    corr_p.first.reserve(corrp.first.size());
    corr_p.second.reserve(corrp.second.size());
    cv::Matx33d K = CM_GetIntrinsics()->K;
    cv::Matx33d K_inv = K.inv();
    cv::Mat E64;
    E.convertTo(E64, CV_64F);
    cv::Matx33d Ex(E64);
    cv::Matx33d F21 = (K_inv.t() * Ex * K_inv).t();
    cv::Matx33d F21_T = F21.t();

    for(size_t i = 0; i < corrp.first.size(); i++)
    {
        //To homog
        cv::Mat firstp = PROJ_ToHomogFromCart(corrp.first[i]);
        cv::Mat secondp = PROJ_ToHomogFromCart(corrp.second[i]);
        cv::Mat epline1 = F21 * firstp;
        fp64 a = epline1.at<double>(0,0);
        fp64 b = epline1.at<double>(1,0);
        fp64 c = epline1.at<double>(2,0);

        fp64 p1 = secondp.at<double>(0,0);
        fp64 p2 = secondp.at<double>(1,0);
        fp64 p3 = secondp.at<double>(2,0);

        fp64 normfactor = (1.0f / sqrt(a*a + b*b));

        fp64 norma = a * normfactor;
        fp64 normb = b * normfactor;
        fp64 normc = c * normfactor;

        fp64 dist1 = abs(norma*p1 + normb*p2 + normc*p3);

        cv::Mat epline2 = F21_T * secondp;

        a = epline2.at<double>(0,0);
        b = epline2.at<double>(1,0);
        c = epline2.at<double>(2,0);

        p1 = firstp.at<double>(0,0);
        p2 = firstp.at<double>(1,0);
        p3 = firstp.at<double>(2,0);

        normfactor = (1.0f / sqrt(a*a + b*b));

        norma = a * normfactor;
        normb = b * normfactor;
        normc = c * normfactor;
        fp64 dist2 = abs(norma*p1 + normb*p2 + normc*p3);
        fp64 dist = (dist1 + dist2) / 2.0f;
        LG_Log(LogSeverity::DBG, "[EP_FindCorrpEpipolar] dist1 = %lf, dist2 = %lf, dist = %lf\n", dist1, dist2, dist);
        if(dist < PANTO_EPIPOLARTRESHOLD)
        {
            corr_p.first.push_back(corrp.first[i]);
            corr_p.second.push_back(corrp.second[i]);
        }
    }
    return corr_p;
}

PointPair2D EP_FilterPointPairByMask(const PointPair2D& corrp, const cv::Mat& mask)
{
    PointPair2D filtered;

    filtered.first.reserve(corrp.first.size());
    filtered.second.reserve(corrp.second.size());

    for (int i = 0; i < mask.rows; ++i) 
    {
        if (mask.at<uchar>(i)) 
        {
            filtered.first.push_back(corrp.first[i]);
            filtered.second.push_back(corrp.second[i]);
        }
    }

    return filtered;
}

void EP_DrawCorrespondences(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2d>& pts1,
        const std::vector<cv::Point2d>& pts2)
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

/*
 *************************************************************
 *************************************************************
 *************************************************************
 *                      Private functions
 *************************************************************
 *************************************************************
 *************************************************************
 * */

cv::Mat __EP_CrossProdMat(cv::Mat x)
{
    assert(x.rows == 3);
    fp64 x1 = x.at<double>(0, 0);
    fp64 x2 = x.at<double>(1, 0);
    fp64 x3 = x.at<double>(2, 0);
    cv::Mat crossprodmat = (cv::Mat_<double>(3,3) <<
            0.0f, -x3, x2, 
            x3, 0.0f, -x1,
            -x2, x1, 0.0f
    );
    return crossprodmat;
}

void __EP_InitAkaze(void)
{
    const fp64 Threshold = OPENCV_AKAZETHRESHOLD;
    LG_Log(LogSeverity::DBG, "[__EP_Init__EP] initing AkazeExtract with %lf\n", Threshold);
    AkazeExtract.akaze = cv::AKAZE::create();
    AkazeExtract.akaze->setThreshold(Threshold);
    AkazeExtract.akaze->setNOctaves(4);
    AkazeExtract.akaze->setNOctaveLayers(4);
    AkazeExtract.matcher = cv::BFMatcher(cv::NORM_HAMMING, false);
    AkazeExtract.matchratio = PANTO_MATCHRATIO;
}

struct MatchesRet __EP_GetMatches(cv::Mat img1, cv::Mat img2)
{

    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] img1 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img1.empty(), img1.rows, img1.cols, img1.type(), img1.channels(), img1.data);

    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] img2 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img2.empty(), img2.rows, img2.cols, img2.type(), img2.channels(), img2.data);
    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] detect and compute img1\n");

    std::vector<cv::KeyPoint> ANMSKeyPoints1 = __EP_Anms(AkazeExtract.akaze, img1);
    cv::Mat ANMSDescriptors1;
    AkazeExtract.akaze->compute(img1, ANMSKeyPoints1, ANMSDescriptors1);

    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] detect and compute img2\n");
    std::vector<cv::KeyPoint> ANMSKeyPoints2 = __EP_Anms(AkazeExtract.akaze, img2);
    cv::Mat ANMSDescriptors2;
    AkazeExtract.akaze->compute(img2, ANMSKeyPoints2, ANMSDescriptors2);

    if(ANMSDescriptors1.empty() || ANMSDescriptors2.empty())
    {
        std::cerr << "No descriptors found in EP_CorrespExtract\n";
        return {};
    }

    struct MatchesRet Ret = __EP_GetCorrespondingPoints(ANMSDescriptors1, ANMSDescriptors2, ANMSKeyPoints1, ANMSKeyPoints2);
    return Ret;
}

DescRet __EP_GetDesc(cv::Mat img)
{
    cv::Mat des;

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] img1 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img.empty(), img.rows, img.cols, img.type(), img.channels(), img.data);

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] detect and compute img\n");
    std::vector<cv::KeyPoint> kp_anms = __EP_Anms(AkazeExtract.akaze, img);
    cv::Mat desanms;
    AkazeExtract.akaze->compute(img, kp_anms, desanms);

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] detect and compute img2\n");

    struct CameraIntrinsics* ci = CM_GetIntrinsics();
    cv::Matx33d K = ci->K;
    cv::Vec<fp64, 5> distcoeffs = ci->distcoeffs;

    if(desanms.empty()) 
    {
        std::cerr << "No descriptors found inf EP_CorrespExtract\n";
        return {};
    }

    std::vector<cv::Point2d> out;

    for (const auto& m : kp_anms) {
        out.push_back(m.pt);
    }

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] num descriptors AkazeExtract = %d\n", out.size());
    std::vector<cv::Point2d> pd;
    cv::undistortPoints(out, pd, K, distcoeffs, cv::noArray(), K);
    out = pd;
    struct DescRet ret;
    ret.Points = out;
    ret.Descriptors = desanms;

    return ret;
}

static struct MatchesRet __EP_GetCorrespondingPoints(cv::Mat Desc1, cv::Mat Desc2, std::vector<cv::KeyPoint> KeyPoints1, std::vector<cv::KeyPoint> KeyPoints2)
{
    std::vector<std::vector<cv::DMatch>> Matches;
    AkazeExtract.matcher.knnMatch(Desc1, Desc2, Matches, 2);

    std::vector<cv::DMatch> GoodMatches;
    for(const auto& m : Matches)
    {
        if(m.size() == 2 && m[0].distance < AkazeExtract.matchratio * m[1].distance)
        {
            GoodMatches.push_back(m[0]);
        }
    }
    PointPair2D RetPoints;
    cv::Mat Desc1Ret, Desc2Ret;
    std::pair<cv::Mat, cv::Mat> DescRet(Desc1Ret, Desc2Ret);

    for (const auto& m : GoodMatches) {
        RetPoints.first.push_back(KeyPoints1[m.queryIdx].pt);
        RetPoints.second.push_back(KeyPoints2[m.trainIdx].pt);
        DescRet.first.push_back(Desc1.row(m.queryIdx));
        DescRet.second.push_back(Desc2.row(m.trainIdx));
    }

    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] num corrp AkazeExtract = %d\n", RetPoints.first.size());
    struct CameraIntrinsics* CI = CM_GetIntrinsics();
    std::vector<cv::Point2d> p1d, p2d;
    cv::undistortPoints(RetPoints.first, p1d, CI->K, CI->distcoeffs, cv::noArray(),  CI->K);
    cv::undistortPoints(RetPoints.second, p2d, CI->K, CI->distcoeffs, cv::noArray(), CI->K);
    RetPoints = PointPair2D(p1d, p2d);
    struct MatchesRet Ret = {
        .Matches = RetPoints,
        .Descriptors = DescRet
    };
    return Ret;
}

static struct MatchesRet __EP_GetCorrespondingPoints(std::pair<cv::Mat, cv::Mat> Descriptors, std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> Points)
{
    std::vector<std::vector<cv::DMatch>> Matches;
    std::vector<cv::Point2d> KeyPoints1 = Points.first;
    std::vector<cv::Point2d> KeyPoints2 = Points.second;
    cv::Mat Desc1 = Descriptors.first;
    cv::Mat Desc2 = Descriptors.second;
    AkazeExtract.matcher.knnMatch(Desc1, Desc2, Matches, 2);

    std::vector<cv::DMatch> GoodMatches(Matches.size());
    for(const auto& m : Matches)
    {
        if(m.size() == 2 && m[0].distance < AkazeExtract.matchratio * m[1].distance)
        {
            GoodMatches.push_back(m[0]);
        }
    }
    PointPair2D RetPoints;
    cv::Mat Desc1Ret, Desc2Ret;
    std::pair<cv::Mat, cv::Mat> DescRet(Desc1Ret, Desc2Ret);

    for (const auto& m : GoodMatches) {
        RetPoints.first.push_back(KeyPoints1[m.queryIdx]);
        RetPoints.second.push_back(KeyPoints2[m.trainIdx]);
        DescRet.first.push_back(Desc1.row(m.queryIdx));
        DescRet.second.push_back(Desc2.row(m.trainIdx));
    }

    LG_Log(LogSeverity::DBG, "[__EP_GetMatches] num corrp AkazeExtract = %d\n", RetPoints.first.size());
    struct CameraIntrinsics* CI = CM_GetIntrinsics();
    std::vector<cv::Point2d> p1d, p2d;
    cv::undistortPoints(RetPoints.first, p1d, CI->K, CI->distcoeffs, cv::noArray(),  CI->K);
    cv::undistortPoints(RetPoints.second, p2d, CI->K, CI->distcoeffs, cv::noArray(), CI->K);
    RetPoints = PointPair2D(p1d, p2d);
    struct MatchesRet Ret = {
        .Matches = RetPoints,
        .Descriptors = DescRet
    };
    return Ret;
}


static std::vector<cv::KeyPoint> __EP_Anms(const cv::Ptr<cv::Feature2D> detector, cv::Mat img)
{
    std::vector<cv::KeyPoint> kp;
    detector->detect(img, kp, cv::noArray());
    std::sort(kp.begin(), kp.end(), [](cv::KeyPoint a, cv::KeyPoint b)
                                    {
                                        return a.response <= b.response;
                                    });
    std::vector<int> anmskp_mask = ssc(kp, kp.size(), 0.2, img.cols, img.rows);
    std::vector<cv::KeyPoint> kp_anms;
    kp_anms.resize(anmskp_mask.size());
    for(std::size_t i = 0; i < anmskp_mask.size(); i++)
    {
        kp_anms[i] = kp[anmskp_mask[i]];
    }
    return kp_anms;
}

#include "../include/EP_CorrespondingPoints.hpp"
#include "EP_CorrespondingPointsPriv.hpp"
#include "CM_Camera.hpp"
#include "LG_Logging.hpp"
#include "PT_Points.hpp"

struct AKAZEExtract AkazeExtract;

struct typeDescriptorTimingStatistics
{
    u64 Count = 0;
    fp64 Sum = 0.0;
    fp64 SumSquared = 0.0;
};

static typeDescriptorTimingStatistics GetDescriptorTotalTiming{};
static typeDescriptorTimingStatistics GetKeyPointsTiming{};
static typeDescriptorTimingStatistics AnmsTiming{};
static typeDescriptorTimingStatistics ComputeDescriptorsTiming{};

static void EPPriv_AddTimingSample(typeDescriptorTimingStatistics& Statistics, const fp64 Time)
{
    Statistics.Count++;
    Statistics.Sum += Time;
    Statistics.SumSquared += Time * Time;
}

static void EPPriv_LogTimingStatistics(const char* Name, const typeDescriptorTimingStatistics& Statistics)
{
    if(Statistics.Count == 0)
    {
        LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] %s: no samples\n", Name);
        return;
    }

    const fp64 Mean = Statistics.Sum / static_cast<fp64>(Statistics.Count);
    const fp64 Variance = std::max<fp64>(
            0.0,
            Statistics.SumSquared / static_cast<fp64>(Statistics.Count) - Mean * Mean);
    const fp64 StandardDeviation = std::sqrt(Variance);

    LG_Log(LogSeverity::DATA,
            "[SLAMTimingSummary] %s: mean = %.6f s, std dev = %.6f s, samples = %llu\n",
            Name,
            Mean,
            StandardDeviation,
            static_cast<unsigned long long>(Statistics.Count));
}

static void EPPriv_RecordGetDescriptorTiming(const fp64 TotalTime, const fp64 GetKeyPointsTime,
        const fp64 AnmsTime, const fp64 ComputeDescriptorsTime)
{
    EPPriv_AddTimingSample(GetDescriptorTotalTiming, TotalTime);
    EPPriv_AddTimingSample(ComputeDescriptorsTiming, ComputeDescriptorsTime);

    if(PANTO_DESCRIPTOR_ANMS)
    {
        EPPriv_AddTimingSample(GetKeyPointsTiming, GetKeyPointsTime);
        EPPriv_AddTimingSample(AnmsTiming, AnmsTime);

        LG_Log(LogSeverity::DBG,
                "[EP_GetDescriptorTiming] total = %.6f s, keypoint detection = %.6f s, ANMS = %.6f s, descriptor computation = %.6f s\n",
                TotalTime,
                GetKeyPointsTime,
                AnmsTime,
                ComputeDescriptorsTime);
    }
    else
    {
        LG_Log(LogSeverity::DBG,
                "[EP_GetDescriptorTiming] total = %.6f s, OpenCV detectAndCompute = %.6f s\n",
                TotalTime,
                ComputeDescriptorsTime);
    }
}

void EP_InitCPointExtractor(void)
{
    __EP_InitAkaze();
}

DescRet EP_GetDescriptors(const cv::Mat& Img)
{
    LG_Log(LogSeverity::DBG, "[EP_GetDescriptors] Getting Descriptors\n");
    DescRet Ret = __EP_GetDesc(Img);
    return Ret;
}

void EP_LogGetDescriptorTimingStatistics(void)
{
    EPPriv_LogTimingStatistics("EP_GetDescriptors/internal total", GetDescriptorTotalTiming);
    if(PANTO_DESCRIPTOR_ANMS)
    {
        EPPriv_LogTimingStatistics("EP_GetDescriptors/keypoint detection", GetKeyPointsTiming);
        EPPriv_LogTimingStatistics("EP_GetDescriptors/ANMS", AnmsTiming);
        EPPriv_LogTimingStatistics("EP_GetDescriptors/descriptor computation", ComputeDescriptorsTiming);
    }
    else
    {
        EPPriv_LogTimingStatistics("EP_GetDescriptors/OpenCV detectAndCompute", ComputeDescriptorsTiming);
    }
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
    const Eigen::Matrix3d& K = CM_GetIntrinsics()->K;
    const Eigen::Matrix3d& K_inv = K.inverse();
    const Eigen::Matrix3d& F21 = K_inv.transpose() * PROJ_CrossProductMatrix(t21) * R21 * K_inv;
    return F21;
}

fp64 EP_CheckEpipolarConstraint(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2,
    const Eigen::Matrix3d& F21, const Eigen::Matrix3d& F12)
{
    const Eigen::Vector3d HomogPoint1 = PROJ_Cart2Homog(Point1);
    const Eigen::Vector3d HomogPoint2 = PROJ_Cart2Homog(Point2);

    Eigen::Vector3d EpipolarLine1 = F12 * HomogPoint2;
    const fp64 EpipolarLine1Norm =
        sqrt(EpipolarLine1.x()*EpipolarLine1.x() + EpipolarLine1.y()*EpipolarLine1.y());

    Eigen::Vector3d EpipolarLine2 = F21 * HomogPoint1;
    const fp64 EpipolarLine2Norm =
        sqrt(EpipolarLine2.x()*EpipolarLine2.x() + EpipolarLine2.y()*EpipolarLine2.y());

    if(EpipolarLine1Norm <= std::numeric_limits<fp64>::epsilon() ||
       EpipolarLine2Norm <= std::numeric_limits<fp64>::epsilon())
    {
        return std::numeric_limits<fp64>::infinity();
    }

    EpipolarLine1 /= EpipolarLine1Norm;
    fp64 Distance1 = std::abs(HomogPoint1.transpose() * EpipolarLine1);

    EpipolarLine2 /= EpipolarLine2Norm;
    fp64 Distance2 = std::abs(HomogPoint2.transpose() * EpipolarLine2);

    fp64 MaxDistance = std::max(Distance1, Distance2);
    return MaxDistance;
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

DescRet __EP_GetDesc(const cv::Mat& img)
{
    const PantoClock::time_point GetDescriptorStartTime = PantoClock::now();

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] img1 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img.empty(), img.rows, img.cols, img.type(), img.channels(), img.data);

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] detect and compute img\n");

    std::vector<cv::KeyPoint> KeyPoints;
    cv::Mat Descriptors;
    fp64 GetKeyPointsTime = 0.0;
    fp64 AnmsTime = 0.0;
    fp64 ComputeDescriptorsTime = 0.0;

    if(PANTO_DESCRIPTOR_ANMS)
    {
        const PantoClock::time_point GetKeyPointsStartTime = PantoClock::now();
        AkazeExtract.akaze->detect(img, KeyPoints, cv::noArray());
        const PantoClock::time_point GetKeyPointsEndTime = PantoClock::now();

        GetKeyPointsTime =
            std::chrono::duration<fp64>(GetKeyPointsEndTime - GetKeyPointsStartTime).count();

        const PantoClock::time_point AnmsStartTime = PantoClock::now();
        KeyPoints = __EP_Anms(KeyPoints, img.cols, img.rows);
        const PantoClock::time_point AnmsEndTime = PantoClock::now();

        AnmsTime =
            std::chrono::duration<fp64>(AnmsEndTime - AnmsStartTime).count();

        const PantoClock::time_point ComputeDescriptorsStartTime = PantoClock::now();
        AkazeExtract.akaze->compute(img, KeyPoints, Descriptors);
        const PantoClock::time_point ComputeDescriptorsEndTime = PantoClock::now();

        ComputeDescriptorsTime =
            std::chrono::duration<fp64>(ComputeDescriptorsEndTime - ComputeDescriptorsStartTime).count();
    }
    else
    {
        const PantoClock::time_point DetectAndComputeStartTime = PantoClock::now();
        AkazeExtract.akaze->detectAndCompute(
                img,
                cv::noArray(),
                KeyPoints,
                Descriptors);
        const PantoClock::time_point DetectAndComputeEndTime = PantoClock::now();

        ComputeDescriptorsTime =
            std::chrono::duration<fp64>(DetectAndComputeEndTime - DetectAndComputeStartTime).count();
    }

    LG_Log(LogSeverity::DBG,
            "[__EP_GetDesc] descriptor ANMS = %d\n",
            PANTO_DESCRIPTOR_ANMS);

    typeCameraIntrinsics* ci = CM_GetIntrinsics();

    cv::Matx33d K(
        ci->K(0, 0), ci->K(0, 1), ci->K(0, 2),
        ci->K(1, 0), ci->K(1, 1), ci->K(1, 2),
        ci->K(2, 0), ci->K(2, 1), ci->K(2, 2));

    cv::Vec<fp64, 5> DistCoeffs(
        ci->k1,
        ci->k2,
        ci->p1,
        ci->p2,
        ci->k3);

    if(Descriptors.empty()) 
    {
        const fp64 GetDescriptorTotalTime =
            std::chrono::duration<fp64>(PantoClock::now() - GetDescriptorStartTime).count();

        EPPriv_RecordGetDescriptorTiming(
                GetDescriptorTotalTime,
                GetKeyPointsTime,
                AnmsTime,
                ComputeDescriptorsTime);

        std::cerr << "No descriptors found inf EP_CorrespExtract\n";
        return {};
    }

    std::vector<cv::Point2d> out;

    for (const auto& m : KeyPoints) {
        out.push_back(m.pt);
    }

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] num descriptors AkazeExtract = %d\n", out.size());
    std::vector<cv::Point2d> pd;
    cv::undistortPoints(out, pd, K, DistCoeffs, cv::noArray(), K);

    std::vector<cv::Point2d> FilteredPoints;
    FilteredPoints.reserve(pd.size());
    cv::Mat FilteredDescriptors;

    for(std::size_t i = 0; i < pd.size(); i++)
    {
        const cv::Point2d& Point = pd[i];
        if(!std::isfinite(Point.x) ||
           !std::isfinite(Point.y) ||
           Point.x < 0.0 ||
           Point.y < 0.0 ||
           Point.x >= static_cast<fp64>(ci->ImageWidth) ||
           Point.y >= static_cast<fp64>(ci->ImageHeight))
        {
            continue;
        }

        FilteredPoints.push_back(Point);
        FilteredDescriptors.push_back(
                Descriptors.row(static_cast<i32>(i)));
    }

    out = std::move(FilteredPoints);
    Descriptors = std::move(FilteredDescriptors);
    struct DescRet ret;
    ret.Points = out;
    ret.Descriptors = Descriptors;

    const fp64 GetDescriptorTotalTime =
        std::chrono::duration<fp64>(PantoClock::now() - GetDescriptorStartTime).count();

    EPPriv_RecordGetDescriptorTiming(
            GetDescriptorTotalTime,
            GetKeyPointsTime,
            AnmsTime,
            ComputeDescriptorsTime);

    return ret;
}


std::vector<cv::KeyPoint> __EP_Anms(std::vector<cv::KeyPoint>& KeyPoints, const i32 ImageColumns, const i32 ImageRows)
{
    if(KeyPoints.empty())
    {
        return {};
    }

    std::sort(KeyPoints.begin(), KeyPoints.end(), [](cv::KeyPoint a, cv::KeyPoint b)
                                    {
                                        return a.response > b.response;
                                    });
    const int NumFeatures = std::min<int>(CV_NFEATURES, KeyPoints.size());
    std::vector<int> anmskp_mask = ssc(KeyPoints, NumFeatures, 0.2, ImageColumns, ImageRows);
    std::vector<cv::KeyPoint> kp_anms;
    kp_anms.resize(anmskp_mask.size());
    for(std::size_t i = 0; i < anmskp_mask.size(); i++)
    {
        kp_anms[i] = KeyPoints[anmskp_mask[i]];
    }
    return kp_anms;
}

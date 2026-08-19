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

DescRet EP_GetDescriptors(const cv::Mat& Img)
{
    LG_Log(LogSeverity::DBG, "[EP_GetDescriptors] Getting Descriptors\n");
    DescRet Ret = __EP_GetDesc(Img);
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
    const Eigen::Matrix3d& K = CM_GetIntrinsics()->K;
    const Eigen::Matrix3d& K_inv = K.inverse();
    const Eigen::Matrix3d& F21 = K_inv.transpose() * PROJ_CrossProductMatrix(t21) * R21 * K_inv;
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
    cv::Mat des;

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] img1 empty=%d rows=%d cols=%d type=%d channels=%d data=%p\n",
       img.empty(), img.rows, img.cols, img.type(), img.channels(), img.data);

    LG_Log(LogSeverity::DBG, "[__EP_GetDesc] detect and compute img\n");
    std::vector<cv::KeyPoint> kp_anms = __EP_Anms(AkazeExtract.akaze, img);
    cv::Mat desanms;
    AkazeExtract.akaze->compute(img, kp_anms, desanms);

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
    cv::undistortPoints(out, pd, K, DistCoeffs, cv::noArray(), K);
    out = pd;
    struct DescRet ret;
    ret.Points = out;
    ret.Descriptors = desanms;

    return ret;
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

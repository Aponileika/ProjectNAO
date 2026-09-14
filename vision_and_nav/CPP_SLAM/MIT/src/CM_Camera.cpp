#include "../include/CM_Camera.hpp"

static bool IntrinsicsSet = false;

inline typeCameraIntrinsics CM_GetConfigIntrinsics(Dataset DatasetID)
{
    switch(DatasetID)
    {
#define X(name, fx_, fy_, s_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, width_, height_, rate_hz_, t_bs_) \
        case Dataset::name: \
        { \
            Eigen::Matrix3d K; \
            Eigen::Matrix4d T_BS; \
            K << \
                fx_,  s_,   cx_, \
                0.0,  fy_,  cy_, \
                0.0,  0.0,  1.0; \
            T_BS << t_bs_; \
            \
            return \
            { \
                .K  = K, \
                .k1 = k1_, \
                .k2 = k2_, \
                .p1 = p1_, \
                .p2 = p2_, \
                .k3 = k3_, \
                .ImageWidth = width_, \
                .ImageHeight = height_, \
                .RateHz = rate_hz_, \
                .T_BS = T_BS \
            }; \
        }

        DATASET_INTRINSICS

#undef X
    }

    return {};
}

static typeCameraIntrinsics ci = CM_GetConfigIntrinsics(panto_dataset);

#if defined(CONFIG_STEREO)
inline typeCameraIntrinsics CM_GetConfigRightIntrinsics(Dataset DatasetID)
{
    switch(DatasetID)
    {
#define X(name, fx_, fy_, s_, cx_, cy_, k1_, k2_, p1_, p2_, k3_, width_, height_, rate_hz_, t_bs_) \
        case Dataset::name: \
        { \
            Eigen::Matrix3d K; \
            Eigen::Matrix4d T_BS; \
            K << \
                fx_,  s_,   cx_, \
                0.0,  fy_,  cy_, \
                0.0,  0.0,  1.0; \
            T_BS << t_bs_; \
            return \
            { \
                .K = K, \
                .k1 = k1_, \
                .k2 = k2_, \
                .p1 = p1_, \
                .p2 = p2_, \
                .k3 = k3_, \
                .ImageWidth = width_, \
                .ImageHeight = height_, \
                .RateHz = rate_hz_, \
                .T_BS = T_BS \
            }; \
        }

        DATASET_STEREO_INTRINSICS

#undef X
        default:
            LG_Log(LogSeverity::ERROR,
                    "[CM_GetConfigRightIntrinsics] Dataset has no stereo intrinsics\n");
            return {};
    }
}

static typeCameraIntrinsics RightIntrinsics =
    CM_GetConfigRightIntrinsics(panto_dataset);
static typeStereoCameraCalibration StereoCalibration{};

static cv::Mat CMPriv_CameraMatrix(const typeCameraIntrinsics& Intrinsics)
{
    return (cv::Mat_<fp64>(3, 3) <<
            Intrinsics.K(0, 0), Intrinsics.K(0, 1), Intrinsics.K(0, 2),
            Intrinsics.K(1, 0), Intrinsics.K(1, 1), Intrinsics.K(1, 2),
            Intrinsics.K(2, 0), Intrinsics.K(2, 1), Intrinsics.K(2, 2));
}

static cv::Mat CMPriv_Distortion(const typeCameraIntrinsics& Intrinsics)
{
    return (cv::Mat_<fp64>(1, 5) <<
            Intrinsics.k1,
            Intrinsics.k2,
            Intrinsics.p1,
            Intrinsics.p2,
            Intrinsics.k3);
}

static void CMPriv_SetStereoCalibration()
{
    if(ci.ImageWidth != RightIntrinsics.ImageWidth ||
       ci.ImageHeight != RightIntrinsics.ImageHeight)
    {
        LG_Log(LogSeverity::ERROR,
                "[CMPriv_SetStereoCalibration] cam0 and cam1 image sizes differ\n");
        return;
    }

    StereoCalibration.T_C1_C0 =
        RightIntrinsics.T_BS.inverse() * ci.T_BS;
    const Eigen::Matrix3d R_C1_C0 =
        StereoCalibration.T_C1_C0.block<3, 3>(0, 0);
    const Eigen::Vector3d t_C1_C0 =
        StereoCalibration.T_C1_C0.block<3, 1>(0, 3);
    StereoCalibration.Baseline = t_C1_C0.norm();

    cv::Mat R_C1_C0CV;
    cv::Mat t_C1_C0CV;
    cv::eigen2cv(R_C1_C0, R_C1_C0CV);
    cv::eigen2cv(t_C1_C0, t_C1_C0CV);

    const cv::Mat K0 = CMPriv_CameraMatrix(ci);
    const cv::Mat D0 = CMPriv_Distortion(ci);
    const cv::Mat K1 = CMPriv_CameraMatrix(RightIntrinsics);
    const cv::Mat D1 = CMPriv_Distortion(RightIntrinsics);
    const cv::Size ImageSize(
            static_cast<i32>(ci.ImageWidth),
            static_cast<i32>(ci.ImageHeight));

    cv::Mat R0Rect;
    cv::Mat R1Rect;
    cv::Mat P0Rect;
    cv::Mat P1Rect;
    cv::Mat Q;
    cv::stereoRectify(
            K0, D0,
            K1, D1,
            ImageSize,
            R_C1_C0CV, t_C1_C0CV,
            R0Rect, R1Rect,
            P0Rect, P1Rect,
            Q,
            cv::CALIB_ZERO_DISPARITY);

    cv::initUndistortRectifyMap(
            K0, D0, R0Rect, P0Rect,
            ImageSize, CV_32FC1,
            StereoCalibration.Map0X,
            StereoCalibration.Map0Y);
    cv::initUndistortRectifyMap(
            K1, D1, R1Rect, P1Rect,
            ImageSize, CV_32FC1,
            StereoCalibration.Map1X,
            StereoCalibration.Map1Y);

    StereoCalibration.K <<
        P0Rect.at<fp64>(0, 0), P0Rect.at<fp64>(0, 1), P0Rect.at<fp64>(0, 2),
        P0Rect.at<fp64>(1, 0), P0Rect.at<fp64>(1, 1), P0Rect.at<fp64>(1, 2),
        P0Rect.at<fp64>(2, 0), P0Rect.at<fp64>(2, 1), P0Rect.at<fp64>(2, 2);
}
#endif

void CM_SetIntrinsics()
{
    if(IntrinsicsSet)return;
    LG_Log(LogSeverity::DBG, "Setting intrinsics\n");
#if defined(CONFIG_STEREO)
    CMPriv_SetStereoCalibration();
#endif
    IntrinsicsSet = true;
    LG_Log(LogSeverity::DBG, "Set Intrinsics\n");
}

typeCameraIntrinsics* CM_GetIntrinsics()
{
    if(!IntrinsicsSet)CM_SetIntrinsics();
    return &ci;
}

#if defined(CONFIG_STEREO)
typeCameraIntrinsics* CM_GetRightIntrinsics()
{
    if(!IntrinsicsSet)CM_SetIntrinsics();
    return &RightIntrinsics;
}

const typeStereoCameraCalibration* CM_GetStereoCalibration()
{
    if(!IntrinsicsSet)CM_SetIntrinsics();
    return &StereoCalibration;
}
#endif

void CM_SetParametrization(typeCamera& Camera)
{
    Camera.Pose.UpdateParametrization();
}

typeCamera CM_CreateCam(Eigen::Matrix3d R, Eigen::Vector3d t, fp64 TimeStamp)
{

    typeCamera Camera
    {
        .Intrinsics = CM_GetIntrinsics(),
        .Pose = typeCameraPose(R, t),
        .TimeStamp = TimeStamp
    };
    return Camera;
}

void CM_SetRtfromParam(typeCamera* Camera)
{
    Camera->Pose.UpdateRt();
}

Eigen::Vector3d CM_GetCameraCenter(const typeCamera& Camera)
{
    return Camera.Pose.GetCameraCenter();
}

typeCamera CM_PredictPose(const typeCameraPose& TPreviousFrame, const typeCameraPose& TPreviousPreviousFrame)
{
    Eigen::Matrix4d TPrevious = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d TPreviousPreviousInverse = Eigen::Matrix4d::Identity();

    TPrevious.block<3, 3>(0, 0) = TPreviousFrame.R;
    TPrevious.block<3, 1>(0, 3) = TPreviousFrame.t;

    const Eigen::Matrix3d& RPreviousPreviousT = TPreviousPreviousFrame.R.transpose();
    TPreviousPreviousInverse.block<3, 3>(0, 0) = RPreviousPreviousT;
    TPreviousPreviousInverse.block<3, 1>(0, 3) = -RPreviousPreviousT*TPreviousPreviousFrame.t;

    const Eigen::Matrix4d& TRelative = TPrevious * TPreviousPreviousInverse;
    const Eigen::Matrix4d& TPredicted = TRelative * TPrevious;

    const Eigen::Matrix3d RPred = static_cast<Eigen::Matrix3d>(TPredicted.block<3,3>(0,0));
    const Eigen::Vector3d tPred = static_cast<Eigen::Vector3d>(TPredicted.block<3,1>(0,3));

    typeCameraPose TPrediction(RPred, tPred);

    typeCameraIntrinsics* Intrinsics = &ci;

    std::string image_name;
    
    typeCamera Prediction = 
    {
        .Intrinsics = Intrinsics,
        .Pose = TPrediction,
        .TimeStamp = PANTO_TIMESTAMP_NOT_SET

    };

    return Prediction;
}

typePose CM_GetBodyToSensor(const typeCamera& Camera)
{
    const Eigen::Matrix3d R = Camera.Intrinsics->T_BS.block<3,3>(0,0);
    const Eigen::Vector3d t = Camera.Intrinsics->T_BS.block<3,1>(0,3);

    return typePose{R, t};
}

typePose CM_GetBodyToSensor(const typeCameraIntrinsics* Intrinsics)
{
    const Eigen::Matrix3d R = Intrinsics->T_BS.block<3,3>(0,0);
    const Eigen::Vector3d t = Intrinsics->T_BS.block<3,1>(0,3);

    return typePose{R, t};
}

#include "../include/CM_Camera.hpp"

static bool IntrinsicsSet = false;

inline typeCameraIntrinsics __CM_GetConfigIntrinsics(Dataset dataset)
{
    switch (dataset)
    {
#define X(name, fx_, fy_, s_, cx_, cy_, k1_, k2_, p1_, p2_, k3_) \
        case Dataset::name:                                      \
            return CameraIntrinsics{                             \
                cv::Matx33d(                                     \
                    fx_,   s_,   cx_,                            \
                    0.0f, fy_,   cy_,                            \
                    0.0f, 0.0f, 1.0f),                           \
                    cv::Vec<fp64, 5>(k1_, k2_, p1_, p2_, k3_)    \
            };
        DATASET_INTRINSICS
#undef X
    }

    return {}; // should never happen
}

static typeCameraIntrinsics ci = __CM_GetConfigIntrinsics(panto_dataset);

void CM_SetIntrinsics()
{
    if(IntrinsicsSet)return;
    LG_Log(LogSeverity::DBG, "Setting intrinsics\n");
    IntrinsicsSet = true;
    LG_Log(LogSeverity::DBG, "Set Intrinsics\n");
}

typeCameraIntrinsics* CM_GetIntrinsics()
{
    if(!IntrinsicsSet)CM_SetIntrinsics();
    return &ci;
}


void CM_SetParametrization(typeCamera& Camera)
{
    /*We use the reparametrization based on
     * R^t[I -t], since the parametrization 
     * [R t] will lead to drastic changes in camera
     * center when t is large and R is small,
     * for example when going straight forward
     * see TSBB33 lecture 5. We parametrize
     * R with unit quaternions, since I cannot
     * seem to find a good implementation of 
     * expm and logm.
     */
    Eigen::Quaterniond q(Camera.Pose.R);
    q.normalize();
    Camera.Parameters.q = q;

    Camera.Parameters.t = -Camera.Pose.R.transpose() * Camera.Pose.t;
}

typeCamera CM_CreateCam(Eigen::Matrix3d R, Eigen::Vector3d t, i32 idx, fp64 TimeStamp)
{
    std::string image_name = "frame" + std::to_string(idx) + ".png";
    typePoseParameters Param = {};
    typeCameraPose Pose = {
        R,
        t
    };

    typeCamera Camera = {
        CM_GetIntrinsics(),
        Pose,
        Param,
        image_name,
        TimeStamp
    };
    CM_SetParametrization(Camera);
    return Camera;
}

void CM_SetRtfromParam(typeCamera& Camera)
{
    const Eigen::Matrix3d RTransposed = Camera.Parameters.q.toRotationMatrix().transpose();
    const Eigen::Vector3d TW2C = -RTransposed * Camera.Parameters.t;
    Camera.Pose.R = RTransposed;
    Camera.Pose.t = TW2C;
}

Eigen::Vector3d CM_GetCameraCenter(const typeCamera& Camera)
{
    return -Camera.Pose.R.transpose() * Camera.Pose.t;
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

    typeCameraPose TPrediction = {
        .R = static_cast<Eigen::Matrix3d>(TPredicted.block<3,3>(0,0)),
        .t = static_cast<Eigen::Vector3d>(TPredicted.block<3,1>(0,3))
    };

    typeCameraIntrinsics* Intrinsics = &ci;
    typePoseParameters Parameters;
    std::string image_name;
    
    typeCamera Prediction = 
    {
        .Intrinsics = Intrinsics,
        .Pose = TPrediction,
        .Parameters = {},
        .image_name = "",
        .TimeStamp = PANTO_TIMESTAMP_NOT_SET

    };

    CM_SetRtfromParam(Prediction);

    return Prediction;
}

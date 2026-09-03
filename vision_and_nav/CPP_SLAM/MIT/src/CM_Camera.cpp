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

#ifndef __CM_CAMERA_HPP_
#define __CM_CAMERA_HPP_
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <Eigen/Geometry>
#include "CArenaAlloc.h"
#include "LG_Logging.hpp"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef struct 
{
    Eigen::Matrix3d K;
    fp64 k1;
    fp64 k2;
    fp64 p1;
    fp64 p2;
    fp64 k3;
    u64 ImageWidth;
    u64 ImageHeight;
    fp64 RateHz;
    Eigen::Matrix4d T_BS;
}typeCameraIntrinsics;

typedef struct 
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
}typePoseParameters;

typedef struct
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
}typeCameraPose;

typedef struct 
{
    //reference to global params
    typeCameraIntrinsics* Intrinsics;
    typeCameraPose Pose;
    typePoseParameters Parameters;
    fp64 TimeStamp;
}typeCamera;


void CM_SetIntrinsics();
typeCameraIntrinsics* CM_GetIntrinsics();
typeCamera CM_CreateCam(Eigen::Matrix3d R, Eigen::Vector3d t, fp64 TimeStamp);
void CM_SetParametrization(typeCamera* cam);
void CM_SetRtfromParam(typeCamera* cam);
Eigen::Vector3d CM_GetCameraCenter(const typeCamera& Camera);
typeCamera CM_PredictPose(const typeCameraPose& TPreviousFrame, const typeCameraPose& TPreviousPreviousFrame);

inline Eigen::Matrix<fp64, 3, 4>CM_GetRt(const typeCamera& CameraPose)
{
    Eigen::Matrix<fp64, 3, 4> Rt;

    Rt.block<3, 3>(0, 0) = CameraPose.Pose.R;
    Rt.col(3) = CameraPose.Pose.t;

    return Rt;
}

#endif //__CM_CAMERA_HPP_

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

typedef struct 
{
    Eigen::Matrix3d K;
    fp64 k1;
    fp64 k2;
    fp64 p1;
    fp64 p2;
    fp64 k3;
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
    std::string image_name;
    fp64 TimeStamp;
}typeCamera;


void CM_SetIntrinsics();
typeCameraIntrinsics* CM_GetIntrinsics();
typeCamera CM_CreateCam(cv::Mat R, cv::Mat t, i32 idx);
void CM_SetParametrization(typeCamera* cam);
void CM_SetRtfromParam(typeCamera* cam);
Eigen::Vector3d CM_GetCameraCenter(const typeCamera& Camera);
typeCamera CM_PredictPose(const typeCameraPose& TPreviousFrame, const typeCameraPose& TPreviousPreviousFrame);

#endif //__CM_CAMERA_HPP_

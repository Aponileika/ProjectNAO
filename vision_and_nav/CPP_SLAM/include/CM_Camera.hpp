#ifndef __CM_CAMERA_HPP_
#define __CM_CAMERA_HPP_
#include <vector>
#include "CArenaAlloc.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

struct CameraIntrinsics
{
    cv::Matx33d K;
    cv::Mat distcoeffs;
};

struct Camera
{
    //reference to global params
    struct CameraIntrinsics* params;
    cv::Mat RigidTransform;
};

extern struct CameraIntrinsics* ci;


void CM_SetIntrinsics(std::string path);
struct CameraIntrinsics* CM_GetIntrinsics();
struct Camera CM_CreateCam(cv::Mat RigidTransform);

#endif //__CM_CAMERA_HPP_

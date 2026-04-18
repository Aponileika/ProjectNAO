#include "CM_Camera.hpp"

struct CameraIntrinsics* ci;
static const fp64 fx = 9.747187409387847*100.0f;
static const fp64 fy = 9.765223334221673*100.0f;
static const fp64 s = 0.0f;
static const fp64 cx = 6.663249058750432*100.0f;
static const fp64 cy = 3.374737864029501*100.0f;

static const fp64 k1 = 6.475901025911835*0.01f;
static const fp64 k2 = -1.903655376657792*0.1f;
static const fp64 k3 = -3.666863513699757*0.001f;
static const fp64 k4 = 2.119531347424837*0.001f;
static const fp64 k5 = 1.113497353924944*0.1f;

void CM_SetIntrinsics(std::string path)
{
    ci = (struct CameraIntrinsics*)malloc(sizeof(struct CameraIntrinsics));
    ci->K = cv::Matx33d(
            fx,   s,    cx,
            0.0f, fy,   cy,
            0.0f, 0.0f, 1.0f);
    ci->distcoeffs = cv::Vec<fp64, 5>(k1, k2, k3, k4, k5);
}

struct CameraIntrinsics* CM_GetIntrinsics()
{
    return ci;
}

struct Camera CM_CreateCam(cv::Mat& RigidTransform)
{
    Camera cam = {
        CM_GetIntrinsics(),
        RigidTransform
    };
    return cam;
}

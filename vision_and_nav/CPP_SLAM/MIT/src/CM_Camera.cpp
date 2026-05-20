#include "../include/CM_Camera.hpp"

struct CameraIntrinsics* ci;
static const fp64 fx = 9.747187409387847*100.0f;
static const fp64 fy = 9.765223334221673*100.0f;
static const fp64 s = 0.0f;
static const fp64 cx = 6.663249058750432*100.0f;
static const fp64 cy = 3.374737864029501*100.0f;

static const fp64 k1 = 6.475901025911835*0.01f;
static const fp64 k2 = -1.903655376657792*0.1f;
static const fp64 p1 = -3.666863513699757*0.001f;
static const fp64 p2 = 2.119531347424837*0.001f;
static const fp64 k3 = 1.113497353924944*0.1f;

static bool IntrinsicsSet = false;

void CM_SetIntrinsics(std::string path)
{
    if(IntrinsicsSet)return;
    LG_Log("Setting intrinsics\n");
    ci = (struct CameraIntrinsics*)malloc(sizeof(struct CameraIntrinsics));
    LG_Log("Setting K\n");
    ci->K = cv::Matx33d(
            fx,   s,    cx,
            0.0f, fy,   cy,
            0.0f, 0.0f, 1.0f);
    LG_Log("Setting distcoeffs\n");
    ci->distcoeffs = cv::Vec<fp64, 5>(k1, k2, p1, p2, k3);
    IntrinsicsSet = true;
    LG_Log("Set Intrinsics\n");
}

struct CameraIntrinsics* CM_GetIntrinsics()
{
    if(!IntrinsicsSet)CM_SetIntrinsics("");
    return ci;
}


void CM_SetParametrization(struct Camera& cam)
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
    Eigen::Matrix3d R;
    cv::cv2eigen(cam.R.t(), R);
    Eigen::Quaterniond q(R);
    q.normalize();
    cam.p->q = q;

    Eigen::Vector3d tm;
    cv::cv2eigen(cam.R.t() * (-cam.t), tm);
    cam.p->t = tm;
}

struct Camera CM_CreateCam(cv::Mat R, cv::Mat t, i32 idx)
{
    std::string path = "./colmap/images/frame" + std::to_string(idx) + ".png";
    Camera cam = {
        CM_GetIntrinsics(),
        R, 
        t,
        (struct Param*)malloc(sizeof(struct Param)),
        path
    };
    CM_SetParametrization(cam);
    return cam;
}

void CM_SetRtfromParam(struct Camera* cam)
{
    Eigen::Matrix3d RE = cam->p->q.toRotationMatrix().transpose();
    cv::Mat R;
    cv::eigen2cv(RE, cam->R);
    
    cv::Mat t;
    Eigen::Vector3d te = -RE * cam->p->t;
    cv::eigen2cv(te, cam->t);
    cam->intrinsics = CM_GetIntrinsics();
}


#include "../include/CM_Camera.hpp"

static bool IntrinsicsSet = false;

inline CameraIntrinsics __CM_GetConfigIntrinsics(Dataset dataset)
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

static struct CameraIntrinsics ci = __CM_GetConfigIntrinsics(panto_dataset);

void CM_SetIntrinsics()
{
    if(IntrinsicsSet)return;
    LG_Log(LogSeverity::DBG, "Setting intrinsics\n");
    IntrinsicsSet = true;
    LG_Log(LogSeverity::DBG, "Set Intrinsics\n");
}

struct CameraIntrinsics* CM_GetIntrinsics()
{
    if(!IntrinsicsSet)CM_SetIntrinsics();
    return &ci;
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
    cv::cv2eigen(-cam.R.t() * cam.t, tm);
    cam.p->t = tm;
}

struct Camera CM_CreateCam(cv::Mat R, cv::Mat t, i32 idx)
{
    std::string image_name = "frame" + std::to_string(idx) + ".png";
    Camera cam = {
        CM_GetIntrinsics(),
        R, 
        t,
        new struct Param{},
        image_name 
    };
    CM_SetParametrization(cam);
    return cam;
}

void CM_SetRtfromParam(struct Camera* cam)
{
    Eigen::Matrix3d R_transposed = cam->p->q.toRotationMatrix().transpose();
    cv::Mat R;
    cv::eigen2cv(R_transposed, cam->R);
    
    cv::Mat t;
    Eigen::Vector3d te = -R_transposed * cam->p->t;
    cv::eigen2cv(te, cam->t);
    cam->intrinsics = CM_GetIntrinsics();
}


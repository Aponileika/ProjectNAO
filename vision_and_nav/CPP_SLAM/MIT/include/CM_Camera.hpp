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


class typePose
{
    public:
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
};

/*We use the reparametrization based on
 * R^t[I -t], since the parametrization
 * [R t] will lead to drastic changes in camera
 * center when t is large and R is small,
 * for example when going straight forward
 * see TSBB33 lecture 5.
 */
class typeCameraPose : public typePose
{
    public:
        Eigen::Quaterniond Quaternion;
        Eigen::Vector3d tParametrization;

        typeCameraPose() = default;

        typeCameraPose(const Eigen::Matrix3d& RNew, const Eigen::Vector3d& tNew)
            : typePose{RNew, tNew},
              Quaternion(RNew.transpose()),
              tParametrization(-RNew.transpose() * tNew)
        {
            Quaternion.normalize();
        };

        void SetPose(const Eigen::Matrix3d& RNew, const Eigen::Vector3d& tNew)
        {
            R = RNew;
            t = tNew;
            Eigen::Quaterniond q(R.transpose());
            q.normalize();
            Quaternion = q;

            tParametrization = -R.transpose() * t;
        }

        void UpdateParametrization(void)
        {
            Eigen::Quaterniond q(R.transpose());
            q.normalize();
            Quaternion = q;

            tParametrization = -R.transpose() * t;
        }

        void UpdateRt(void)
        {
            const Eigen::Matrix3d RTransposed = Quaternion.toRotationMatrix().transpose();
            const Eigen::Vector3d TW2C = -RTransposed * tParametrization;
            R = RTransposed;
            t = TW2C;
        }

        const Eigen::Vector3d GetCameraCenter(void) const
        {
            return tParametrization;
        }
};

typedef struct 
{
    //reference to global params
    typeCameraIntrinsics* Intrinsics;
    typeCameraPose Pose;
    fp64 TimeStamp;
}typeCamera;


void CM_SetIntrinsics();
typeCameraIntrinsics* CM_GetIntrinsics();
typeCamera CM_CreateCam(Eigen::Matrix3d R, Eigen::Vector3d t, fp64 TimeStamp);
void CM_SetParametrization(typeCamera* cam);
void CM_SetRtfromParam(typeCamera* cam);
Eigen::Vector3d CM_GetCameraCenter(const typeCamera& Camera);
typeCamera CM_PredictPose(const typeCameraPose& TPreviousFrame, const typeCameraPose& TPreviousPreviousFrame);
typePose CM_GetBodyToSensor(const typeCamera& Camera);
typePose CM_GetBodyToSensor(const typeCameraIntrinsics* Intrinsics);

inline Eigen::Matrix<fp64, 3, 4>CM_GetRt(const typeCamera& CameraPose)
{
    Eigen::Matrix<fp64, 3, 4> Rt;

    Rt.block<3, 3>(0, 0) = CameraPose.Pose.R;
    Rt.col(3) = CameraPose.Pose.t;

    return Rt;
}

#endif //__CM_CAMERA_HPP_

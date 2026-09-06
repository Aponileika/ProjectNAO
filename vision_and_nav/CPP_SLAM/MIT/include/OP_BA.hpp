#ifndef __OP_BA_HPP_
#define __OP_BA_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "IMU_PreIntegration.hpp"
#include "MAP_Mapping.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef enum 
{
    OptimizationTypePoseAndPoints = 0,
    OptimizationTypePose = 1,
    OptimizationTypeTracking = 2,
    OptimizationTypeLocal = 3
}typeOptimizationTarget;

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

struct typeOPCameraIntrinsics
{
    typeOPCameraIntrinsics(Eigen::Matrix3d K) :
        fx(K(0, 0)), fy(K(1, 1)), cx(K(0, 2)), cy(K(1, 2)) {}
     fp64 fx, fy, cx, cy;
};

struct OP_ReprojectionError
{
    OP_ReprojectionError(fp64 ObservedX, fp64 ObservedY, const struct typeOPCameraIntrinsics* Intrinsics)
        : ObservedX_(ObservedX), ObservedY_(ObservedY), Intrinsics_(Intrinsics){}

    template <typename T>
    bool operator()(const T* const qp,
                    const T* const tp,
                    const T* const Xp, 
                    T* residuals) const{
        Eigen::Map<const Eigen::Quaternion<T>> qwc(qp);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(tp);
        Eigen::Map<const Eigen::Matrix<T,4,1>> X(Xp);

        //Note we do not use homogenous coordinates to project, 
        //Right now we distort all projected points, a more reasonable
        //approach could be to distort all detected ORB keypoints instead
        //Since this would save some computations

        //IREG P.176, since rotation is parametrized as 
        //R^T, and translation as -R^T*t.
        const Eigen::Quaternion<T> qcw = qwc.conjugate();

        const fp64 fx_ = Intrinsics_->fx;
        const fp64 fy_ = Intrinsics_->fy;
        const fp64 cx_ = Intrinsics_->cx;
        const fp64 cy_ = Intrinsics_->cy;

        //X in camera coordinates is now R^T(X - t)
        Eigen::Matrix<T, 3, 1> XYZ(
                X(0),
                X(1),
                X(2));
        T W = X(3);
        Eigen::Matrix<T, 3, 1> Xc = qcw * (XYZ - t * W);

        //projection
        T x = Xc.x() / Xc.z();
        T y = Xc.y() / Xc.z();

        //Normalized image coordinates, u,v convention from graphics programming?
        T u = T(fx_) * x + T(cx_);
        T v = T(fy_) * y + T(cy_);

        residuals[0] = u - T(ObservedX_);
        residuals[1] = v - T(ObservedY_);
        return true;
    }
    static ceres::CostFunction* Create(const fp64 ObservedX, const fp64 ObservedY, const struct typeOPCameraIntrinsics* Intrinsics)
    {
        return new ceres::AutoDiffCostFunction<OP_ReprojectionError, 2, 4, 3, 4>(
            new OP_ReprojectionError(ObservedX, ObservedY, Intrinsics));
    }

    fp64 ObservedX_, ObservedY_;
    const struct typeOPCameraIntrinsics* Intrinsics_;
};

struct OP_IMUResidual
{
    typePreIntegrationData PreIntegration;
    Eigen::Matrix<fp64, 15, 15> SqrtInformation;
    Eigen::Vector3d Gravity;
    Eigen::Quaterniond CameraToBodyRotation;
    Eigen::Vector3d CameraToBodyTranslation;

    OP_IMUResidual(const typePreIntegrationData& PreIntegrationData, const Eigen::Matrix<fp64, 15, 15>& SqrtInfo, const Eigen::Vector3d& Grav,
            const Eigen::Matrix4d* TBS)
        : PreIntegration(PreIntegrationData), SqrtInformation(SqrtInfo), Gravity(Grav),
          CameraToBodyRotation(TBS->block<3,3>(0,0)), CameraToBodyTranslation(TBS->block<3,1>(0,3))
    {
        // kind of not necessary, eigen does this.
        CameraToBodyRotation.normalize();
    }

    template <typename T> bool operator()(
                const T* const Quaternion_i,
                const T* const Pose_i,
                const T* const Velocity_i,
                const T* const GyroBias_i,
                const T* const AccelorometerBias_i,

                const T* const Quaternion_j,
                const T* const Pose_j,
                const T* const Velocity_j,
                const T* const GyroBias_j,
                const T* const AccelorometerBias_j,

                T* residuals) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> Qi(Quaternion_i);
        Eigen::Map<const Eigen::Matrix<T,3,1>> Posei(Pose_i);
        Eigen::Map<const Eigen::Matrix<T,3,1>> Velocityi(Velocity_i);
        Eigen::Map<const Eigen::Matrix<T,3,1>> GyroBiasi(GyroBias_i);
        Eigen::Map<const Eigen::Matrix<T,3,1>> AccBiasi(AccelorometerBias_i);

        Eigen::Map<const Eigen::Quaternion<T>> Qj(Quaternion_j);
        Eigen::Map<const Eigen::Matrix<T,3,1>> Posej(Pose_j);
        Eigen::Map<const Eigen::Matrix<T,3,1>> Velocityj(Velocity_j);
        Eigen::Map<const Eigen::Matrix<T,3,1>> GyroBiasj(GyroBias_j);
        Eigen::Map<const Eigen::Matrix<T,3,1>> AccBiasj(AccelorometerBias_j);

        const T Dt = T(PreIntegration.DeltaT);

        const Eigen::Matrix<T,3,1> GravityT =
            Gravity.template cast<T>();

        // --------------------------------
        // Bias correction
        // --------------------------------

        const Eigen::Matrix<T,3,1> DeltaBg =
            GyroBiasi - PreIntegration.GyroBias.template cast<T>();

        const Eigen::Matrix<T,3,1> DeltaBa =
            AccBiasi - PreIntegration.AccelBias.template cast<T>();

        const Sophus::SO3<T> DeltaRNominal(
                Eigen::Quaternion<T>( PreIntegration.DeltaR.template cast<T>()));

        const Sophus::SO3<T> DeltaRCorrected = DeltaRNominal * Sophus::SO3<T>::exp( PreIntegration.JRg.template cast<T>() *
                    DeltaBg);

        const Eigen::Matrix<T,3,1> DeltaVCorrected =
            PreIntegration.DeltaVelocity.template cast<T>() + PreIntegration.JVg.template cast<T>() * DeltaBg +
            PreIntegration.JVa.template cast<T>() * DeltaBa;

        const Eigen::Matrix<T,3,1> DeltaPCorrected =
            PreIntegration.DeltaPosition.template cast<T>() + PreIntegration.JPg.template cast<T>() * DeltaBg +
            PreIntegration.JPa.template cast<T>() * DeltaBa;

        // --------------------------------
        // Camera pose -> body pose
        // --------------------------------

        const Eigen::Quaternion<T> QBC = CameraToBodyRotation.template cast<T>();
        const Eigen::Matrix<T,3,1> tBC = CameraToBodyTranslation.template cast<T>();
        const Eigen::Quaternion<T> QWBi = Qi * QBC.conjugate();
        const Eigen::Quaternion<T> QWBj = Qj * QBC.conjugate();
        const Eigen::Matrix<T,3,1> PWBi = Posei - QWBi * tBC;
        const Eigen::Matrix<T,3,1> PWBj = Posej - QWBj * tBC;

        const Sophus::SO3<T> RWBi(QWBi);
        const Sophus::SO3<T> RWBj(QWBj);

        const Eigen::Matrix<T,3,1> RotationResidual = (DeltaRCorrected.inverse() * RWBi.inverse() * RWBj).log();
        const Eigen::Matrix<T,3,1> VelocityResidual = QWBi.conjugate() * ( Velocityj - Velocityi - GravityT * Dt) - DeltaVCorrected;
        const Eigen::Matrix<T,3,1> PositionResidual = QWBi.conjugate() * ( PWBj - PWBi - Velocityi * Dt - T(0.5) * GravityT * Dt * Dt) -
            DeltaPCorrected;
        const Eigen::Matrix<T,3,1> GyroBiasResidual = GyroBiasj - GyroBiasi;
        const Eigen::Matrix<T,3,1> AccBiasResidual = AccBiasj - AccBiasi;

        Eigen::Matrix<T,15,1> Residual;
        Residual.template segment<3>(0)  = RotationResidual;
        Residual.template segment<3>(3)  = VelocityResidual;
        Residual.template segment<3>(6)  = PositionResidual;
        Residual.template segment<3>(9)  = GyroBiasResidual;
        Residual.template segment<3>(12) = AccBiasResidual;
        Eigen::Map<Eigen::Matrix<T,15,1>> Output(residuals);
        Output = SqrtInformation.template cast<T>() * Residual;
        return true;
    }

    static ceres::CostFunction* Create(const typePreIntegrationData PreIntegrationData, const Eigen::Matrix<fp64, 15, 15> SqrtInfo, const Eigen::Vector3d Grav,
            const Eigen::Matrix4d* TBS)
    {
        return new ceres::AutoDiffCostFunction<OP_IMUResidual, 
               15,

               4, 3, 3, 3, 3,
               4, 3, 3, 3, 3
                   >(
                new OP_IMUResidual(PreIntegrationData, SqrtInfo, Grav, TBS));
    }

};

void OP_BundleAdjust(typeGlobalMap* Map, typeOptimizationTarget Target,
        const typeLocalMap& LocalMap, typeKeyFrame* NewKeyFrame,
        typeKeyFrame* PreviousFrame = nullptr);

#endif //__OP_BA_HPP_

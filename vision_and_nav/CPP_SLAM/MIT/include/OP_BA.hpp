#ifndef __OP_BA_HPP_
#define __OP_BA_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <unordered_map>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "IMU_PreIntegration.hpp"
#include "LG_Logging.hpp"
#include "MAP_Mapping.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef enum 
{
    OptimizationTypePoseAndPoints = 0,
    OptimizationTypePose = 1,
    OptimizationTypeTracking = 2,
    OptimizationTypeLocal = 3,
    OptimizationTypePointsOnly = 4
}typeOptimizationTarget;

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

#if defined(CONFIG_STEREO)
struct typeOPStereoIntrinsics
{
    typeOPStereoIntrinsics(typeStereoCameraCalibration StereoCalib) :
        fx(StereoCalib.K(0, 0)), fy(StereoCalib.K(1, 1)), cx(StereoCalib.K(0, 2)), cy(StereoCalib.K(1, 2)),
                B(StereoCalib.Baseline) {}
     fp64 fx, fy, cx, cy, B;
};
#endif

struct typeOPCameraIntrinsics
{
    typeOPCameraIntrinsics(Eigen::Matrix3d K) :
        fx(K(0, 0)), fy(K(1, 1)), cx(K(0, 2)), cy(K(1, 2)) {}
     fp64 fx, fy, cx, cy;
};

#if defined(CONFIG_STEREO)
struct OP_StereoReprojectionError
{
    OP_StereoReprojectionError(fp64 ObservedXLeft, fp64 ObservedYLeft, fp64 ObservedXRight, fp64 ObservedYRight, 
            const struct typeOPStereoIntrinsics* Intrinsics)
        : ObservedXLeft_(ObservedXLeft), ObservedYLeft_(ObservedYLeft), 
          ObservedXRight_(ObservedXRight), ObservedYRight_(ObservedYRight), 
        Intrinsics_(Intrinsics){}

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
        const Eigen::Vector3d Bt(-Intrinsics_->B , 0.0 , 0.0);

        //X in camera coordinates is now R^T(X - t)
        Eigen::Matrix<T, 3, 1> XYZ(
                X(0),
                X(1),
                X(2));
        T W = X(3);
        Eigen::Matrix<T, 3, 1> Xc = qcw * (XYZ - t * W);
        Eigen::Matrix<T, 3, 1> XcRight = Xc + Bt * W;

        //projection
        T x = Xc.x() / Xc.z();
        T y = Xc.y() / Xc.z();

        T xRight = XcRight.x() / Xc.z();
        T yRight = XcRight.y() / Xc.z();

        //Normalized image coordinates, u,v convention from graphics programming?
        T u = T(fx_) * x + T(cx_);
        T v = T(fy_) * y + T(cy_);

        T uRight = T(fx_) * xRight + T(cx_);
        T vRight = T(fy_) * yRight + T(cy_);

        residuals[0] = u - T(ObservedXLeft_);
        residuals[1] = v - T(ObservedYLeft_);

        residuals[2] = uRight - T(ObservedXRight_);
        residuals[3] = vRight - T(ObservedYRight_);
        return true;
    }

    static ceres::CostFunction* Create(const fp64 ObservedXLeft, const fp64 ObservedYLeft, const fp64 ObservedXRight, const fp64 ObservedYRight,
            const struct typeOPStereoIntrinsics* Intrinsics)
    {
        return new ceres::AutoDiffCostFunction<OP_StereoReprojectionError, 4, 4, 3, 4>(
            new OP_StereoReprojectionError(ObservedXLeft, ObservedYLeft, ObservedXRight, ObservedYRight, Intrinsics));
    }

    fp64 ObservedXLeft_, ObservedYLeft_;
    fp64 ObservedXRight_, ObservedYRight_;
    const struct typeOPStereoIntrinsics* Intrinsics_;
};
#endif // CONFIG_STEREO

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

        const Eigen::Matrix<T,3,1> GravityT = Gravity.template cast<T>();

        // --------------------------------
        // Bias correction
        // --------------------------------

        const Eigen::Matrix<T,3,1> DeltaBg =
            GyroBiasi - PreIntegration.GyroBias.template cast<T>();

        const Eigen::Matrix<T,3,1> DeltaBa =
            AccBiasi - PreIntegration.AccelBias.template cast<T>();

        const Sophus::SO3<T> DeltaRNominal(
                Eigen::Quaternion<T>(PreIntegration.DeltaR.template cast<T>()));

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

class OP_LocalBAAbortCallback : public ceres::IterationCallback
{
public:
    OP_LocalBAAbortCallback(
        const std::atomic<u64>& Generation,
        const u64 StartGeneration)
        : Generation(Generation),
          StartGeneration(StartGeneration)
    {
    }

    ceres::CallbackReturnType operator()(
        const ceres::IterationSummary&) override
    {
        if(Generation.load(std::memory_order_relaxed)
                != StartGeneration)
        {
            WasAborted = true;
            return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
        }

        return ceres::SOLVER_CONTINUE;
    }

    bool WasAborted = false;

private:
    const std::atomic<u64>& Generation;
    const u64 StartGeneration;
};

struct OP_EigenValueBiasResidual
{
    /*
     * One residual per consecutive KF, with all of them shared gyrobias. Initialize
     * the gyrobias to 0
     * */
    Eigen::Matrix3d BodyToCameraRotation;
    Eigen::Matrix3d CameraToBodyRotation;
    Eigen::Matrix3d _GyroJacobian;
    Eigen::Quaterniond PreintegratedRotation;
    Eigen::Matrix3d Sxx, Syy, Szz, Sxy, Syz, Szx;

    OP_EigenValueBiasResidual(const Eigen::Matrix3d& DeltaR, const Eigen::Matrix3d& GyroJacobian,  const Eigen::Matrix4d* TBS, 
            const Eigen::Matrix3d& _Sxx, const Eigen::Matrix3d& _Syy, const Eigen::Matrix3d& _Szz, const Eigen::Matrix3d& _Sxy, 
            const Eigen::Matrix3d& _Syz, const Eigen::Matrix3d& _Szx) : 
        _GyroJacobian(GyroJacobian), BodyToCameraRotation(TBS->block<3,3>(0,0)), PreintegratedRotation(Eigen::Quaterniond(DeltaR)),
        Sxx(_Sxx), Syy(_Syy), Szz(_Szz), Sxy(_Sxy), Syz(_Syz), Szx(_Szx)
    {
        CameraToBodyRotation = BodyToCameraRotation.transpose();
    }

    template <typename T>
        bool operator()(
                const T* const GyroBias, 
                T* Residuals) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> Bias(GyroBias);

            const Sophus::SO3<T> Gamma(
                    PreintegratedRotation.template cast<T>());

            const Sophus::SO3<T> BiasCorrectedGamma =
                Gamma * Sophus::SO3<T>::exp( _GyroJacobian.template cast<T>() * Bias);

            const Eigen::Matrix<T, 3, 3> CameraRotation =
                BodyToCameraRotation.template cast<T>() *
                BiasCorrectedGamma.matrix() *
                CameraToBodyRotation.template cast<T>();

            const Eigen::Matrix<T, 1, 3> R0 = CameraRotation.row(0);
            const Eigen::Matrix<T, 1, 3> R1 = CameraRotation.row(1);
            const Eigen::Matrix<T, 1, 3> R2 = CameraRotation.row(2);

            const Eigen::Matrix<T, 3, 3> SxxT = Sxx.template cast<T>();
            const Eigen::Matrix<T, 3, 3> SyyT = Syy.template cast<T>();
            const Eigen::Matrix<T, 3, 3> SzzT = Szz.template cast<T>();
            const Eigen::Matrix<T, 3, 3> SxyT = Sxy.template cast<T>();
            const Eigen::Matrix<T, 3, 3> SyzT = Syz.template cast<T>();
            const Eigen::Matrix<T, 3, 3> SzxT = Szx.template cast<T>();

            Eigen::Matrix<T, 3, 3> M;

            M(0, 0) =
                (R2 * SyyT * R2.transpose())(0, 0)
                - T(2) * (R2 * SyzT * R1.transpose())(0, 0)
                + (R1 * SzzT * R1.transpose())(0, 0);

            M(0, 1) =
                (R2 * SyzT * R0.transpose())(0, 0)
                - (R2 * SxyT * R2.transpose())(0, 0)
                - (R1 * SzzT * R0.transpose())(0, 0)
                + (R1 * SzxT * R2.transpose())(0, 0);

            M(0, 2) =
                (R2 * SxyT * R1.transpose())(0, 0)
                - (R2 * SyyT * R0.transpose())(0, 0)
                - (R1 * SzxT * R1.transpose())(0, 0)
                + (R1 * SyzT * R0.transpose())(0, 0);

            M(1, 1) =
                (R0 * SzzT * R0.transpose())(0, 0)
                - T(2) * (R0 * SzxT * R2.transpose())(0, 0)
                + (R2 * SxxT * R2.transpose())(0, 0);

            M(1, 2) =
                (R0 * SzxT * R1.transpose())(0, 0)
                - (R0 * SyzT * R0.transpose())(0, 0)
                - (R2 * SxxT * R1.transpose())(0, 0)
                + (R2 * SxyT * R0.transpose())(0, 0);

            M(2, 2) =
                (R1 * SxxT * R1.transpose())(0, 0)
                - T(2) * (R0 * SxyT * R1.transpose())(0, 0)
                + (R0 * SyyT * R0.transpose())(0, 0);

            // M is symmetric.
            M(1, 0) = M(0, 1);
            M(2, 0) = M(0, 2);
            M(2, 1) = M(1, 2);

            // Calculate lambda_min(M).
            // Characteristic polynomial:
            // lambda^3 + b*lambda^2 + c*lambda + d = 0
            // Using closed form solution to be differentiable with jets
            const T b = -M(0, 0) - M(1, 1) - M(2, 2);

            const T c =
                M(0, 0) * M(1, 1) +
                M(0, 0) * M(2, 2) +
                M(1, 1) * M(2, 2) -
                M(0, 1) * M(0, 1) -
                M(0, 2) * M(0, 2) -
                M(1, 2) * M(1, 2);

            const T d =
                M(1, 1) * M(0, 2) * M(0, 2) +
                M(0, 0) * M(1, 2) * M(1, 2) +
                M(2, 2) * M(0, 1) * M(0, 1) -
                M(0, 0) * M(1, 1) * M(2, 2) -
                T(2) * M(0, 1) * M(1, 2) * M(0, 2);

            const T s =
                T(2) * b * b * b -
                T(9) * b * c +
                T(27) * d;

            const T discriminantTerm =
                b * b - T(3) * c;

            const T t = T(4) * discriminantTerm *
                discriminantTerm *
                discriminantTerm;

            const T alpha = ceres::acos(s / ceres::sqrt(t));
            const T beta = alpha / T(3);

            const T r = T(0.5) * ceres::sqrt(t);
            const T w = ceres::pow(r, 1.0 / 3.0);

            Residuals[0] =
                (-b - T(2) * w * ceres::cos(beta)) / T(3);

            return true;
        }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& DeltaR, const Eigen::Matrix3d& GyroJacobian, const Eigen::Matrix4d* TBS,
            const Eigen::Matrix3d& _Sxx, const Eigen::Matrix3d& _Syy, const Eigen::Matrix3d& _Szz, const Eigen::Matrix3d& _Sxy, 
            const Eigen::Matrix3d& _Syz, const Eigen::Matrix3d& _Szx
            )
    {
        return new ceres::AutoDiffCostFunction<OP_EigenValueBiasResidual, 1, 3>(
                new OP_EigenValueBiasResidual(DeltaR, GyroJacobian, TBS,
                                              _Sxx, _Syy, _Szz, _Sxy, _Syz, _Szx
                    ));
    }
};

struct OP_MAPGyroVelocityBiasGravityAndVelocity
{
    Eigen::Matrix3d Rwb;
    Eigen::Matrix3d RwbNext;
    Eigen::Vector3d Position;
    Eigen::Vector3d PositionNext;
    fp64 DeltaTime;

    Eigen::Vector3d PreintegratedPosition;
    Eigen::Vector3d PreintegratedVelocity;
    Eigen::Matrix3d PreintegratedRotation;

    Eigen::Vector3d PreintegrationGyroBias;
    Eigen::Vector3d PreintegrationAccBias;

    Eigen::Matrix3d JAlphaBg;
    Eigen::Matrix3d JAlphaBa;
    Eigen::Matrix3d JBetaBg;
    Eigen::Matrix3d JBetaBa;
    Eigen::Matrix3d JGammaBg;

    Eigen::Matrix<fp64, 9, 9> SqrtInformation;

    OP_MAPGyroVelocityBiasGravityAndVelocity(
            const Eigen::Matrix3d& _Rwb,
            const Eigen::Matrix3d& _RwbNext,
            const Eigen::Vector3d& _Position,
            const Eigen::Vector3d& _PositionNext,
            const fp64 _DeltaTime,

            const Eigen::Vector3d& _PreintegratedPosition,
            const Eigen::Vector3d& _PreintegratedVelocity,
            const Eigen::Matrix3d& _PreintegratedRotation,
            const Eigen::Vector3d& _PreintegrationGyroBias,
            const Eigen::Vector3d& _PreintegrationAccBias,

            const Eigen::Matrix3d& _JAlphaBg,
            const Eigen::Matrix3d& _JAlphaBa,
            const Eigen::Matrix3d& _JBetaBg,
            const Eigen::Matrix3d& _JBetaBa,
            const Eigen::Matrix3d& _JGammaBg,
            const Eigen::Matrix<fp64, 9, 9> _SqrtInformation
            ) 
        : Rwb(_Rwb),
        RwbNext(_RwbNext),
        Position(_Position),
        PositionNext(_PositionNext),
        DeltaTime(_DeltaTime),

        PreintegratedPosition(_PreintegratedPosition),
        PreintegratedVelocity(_PreintegratedVelocity),
        PreintegratedRotation(_PreintegratedRotation),

        PreintegrationGyroBias(_PreintegrationGyroBias),
        PreintegrationAccBias(_PreintegrationAccBias),

        JAlphaBg(_JAlphaBg),
        JAlphaBa(_JAlphaBa),
        JBetaBg(_JBetaBg),
        JBetaBa(_JBetaBa),
        JGammaBg(_JGammaBg),
        SqrtInformation(_SqrtInformation)
    {}

    template <typename T> bool operator()(
         const T* const Velocity,
         const T* const VelocityNext,
         const T* const GravityWorldZAlignmentRotation,
         const T* const GyroBias,
         const T* const AccelorometerBias,
         T* Residuals) const
        {
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> V(Velocity);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> VNext(VelocityNext);
            const Eigen::Map<const Eigen::Quaternion<T>> GravityWorldAlign(GravityWorldZAlignmentRotation);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> GBias(GyroBias);
            const Eigen::Map<const Eigen::Matrix<T, 3, 1>> ABias(AccelorometerBias);

            const Eigen::Matrix<T, 3, 1> DeltaBg = GBias - PreintegrationGyroBias.template cast<T>();

            const Eigen::Matrix<T, 3, 1> DeltaBa = ABias - PreintegrationAccBias.template cast<T>();

            const Eigen::Matrix<T, 3, 1> AlphaCorrected = PreintegratedPosition.template cast<T>() +
                JAlphaBg.template cast<T>() * DeltaBg + JAlphaBa.template cast<T>() * DeltaBa;

            const Eigen::Matrix<T, 3, 1> BetaCorrected = PreintegratedVelocity.template cast<T>() +
                JBetaBg.template cast<T>() * DeltaBg + JBetaBa.template cast<T>() * DeltaBa;

            const Sophus::SO3<T> Gamma( PreintegratedRotation.template cast<T>());

            const Sophus::SO3<T> GammaCorrected = Gamma * Sophus::SO3<T>::exp(JGammaBg.template cast<T>() * DeltaBg);


            const T Dt = T(DeltaTime);

            const Eigen::Matrix<T, 3, 3> Rk = Rwb.template cast<T>();
            const Eigen::Matrix<T, 3, 3> RkNext = RwbNext.template cast<T>();
            const Eigen::Matrix<T, 3, 1> Pk = Position.template cast<T>();
            const Eigen::Matrix<T, 3, 1> PkNext = PositionNext.template cast<T>();
            const Eigen::Quaternion<T> GravityQuaternion = GravityWorldAlign.normalized();
            const Eigen::Matrix<T, 3, 1> Gravity = GravityQuaternion * Eigen::Matrix<T, 3, 1>(T(0), T(0), T(9.81));
            const Eigen::Matrix<T, 3, 1> PositionResidual = Rk.transpose() *
                (PkNext - Pk - V * Dt - T(0.5) * Gravity * Dt * Dt) - AlphaCorrected;

            const Eigen::Matrix<T, 3, 1> VelocityResidual = Rk.transpose() *
                (VNext - V - Gravity * Dt) - BetaCorrected;

            const Sophus::SO3<T> VisualRelativeRotation(Rk.transpose() * RkNext);

            const Eigen::Matrix<T, 3, 1> RotationResidual =
                (GammaCorrected.inverse() * VisualRelativeRotation).log();

            Eigen::Map<Eigen::Matrix<T, 9, 1>> Res(Residuals);

            Res.template segment<3>(0) = PositionResidual;
            Res.template segment<3>(3) = VelocityResidual;
            Res.template segment<3>(6) = RotationResidual;

            Res = SqrtInformation.template cast<T>() * Res;

            return true;
        }

    static ceres::CostFunction* Create(
            const Eigen::Matrix3d& _Rwb,
            const Eigen::Matrix3d& _RwbNext,
            const Eigen::Vector3d& _Position,
            const Eigen::Vector3d& _PositionNext,
            const fp32 _DeltaTime,
            const Eigen::Vector3d& _PreintegratedPosition,
            const Eigen::Vector3d& _PreintegratedVelocity,
            const Eigen::Matrix3d& _PreintegratedRotation,
            const Eigen::Vector3d& _PreintegrationGyroBias,
            const Eigen::Vector3d& _PreintegrationAccBias,
            const Eigen::Matrix3d& _JAlphaBg,
            const Eigen::Matrix3d& _JAlphaBa,
            const Eigen::Matrix3d& _JBetaBg,
            const Eigen::Matrix3d& _JBetaBa,
            const Eigen::Matrix3d& _JGammaBg,
            const Eigen::Matrix<fp64, 9, 9>& _SqrtInformation
    )
    {
        return new ceres::AutoDiffCostFunction<OP_MAPGyroVelocityBiasGravityAndVelocity, 
            9, 
            3, 3, 4, 3, 3>(
                new OP_MAPGyroVelocityBiasGravityAndVelocity(
                    _Rwb,
                    _RwbNext,
                    _Position,
                    _PositionNext,
                    _DeltaTime,
                    _PreintegratedPosition,
                    _PreintegratedVelocity,
                    _PreintegratedRotation,
                    _PreintegrationGyroBias,
                    _PreintegrationAccBias,
                    _JAlphaBg,
                    _JAlphaBa,
                    _JBetaBg,
                    _JBetaBa,
                    _JGammaBg,
                    _SqrtInformation
                )
            );
    }
};

bool OP_BundleAdjust(typeGlobalMap& Map, typeOptimizationTarget Target,
        const typeLocalMap& LocalMap, typeKeyFrame* NewKeyFrame,
        typeKeyFrame* PreviousFrame = nullptr,
        typeKeyFrameQueue* KeyFrameQueue = nullptr);

bool OP_BundleAdjustTracking(typeLocalMapTracking& TrackingMap,
        typeKeyFrame* NewKeyFrame, typeKeyFrame* PreviousFrame);
bool OP_BundleAdjustTrackingVisualOnly(typeLocalMapTracking& TrackingMap,
        typeKeyFrame* NewKeyFrame, typeKeyFrame* PreviousFrame);

bool OP_BundleAdjustLocal(typeLocalMap& LocalMap, typeKeyFrameQueue* KeyFrameQueue);
bool OP_BundleAdjustLocalVisualOnly(typeLocalMap& LocalMap, typeKeyFrameQueue* KeyFrameQueue);

bool OP_BundleAdjustLocalVisualOnly(typeLocalMap& LocalMap,
        typeKeyFrameQueue* KeyFrameQueue);

// This local map should just be all global kfs and mappoints at this point.
bool OP_EigenValueSolveGyroBias(typeGlobalMap& LocalMap);
bool OP_MAPGyroVelocityBiasGravityAndVelocity(typePantoVector<typeKeyFrame>& LocalMap);

#endif //__OP_BA_HPP_

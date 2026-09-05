#ifndef IMU_PREINTEGRATION_HPP
#define IMU_PREINTEGRATION_HPP
#include <Eigen/Dense>
#include "CArenaAlloc.h"
#include "IMU_IMUReader.hpp"
#include "sophus/so3.hpp"

class typeNavigationState
{
    public:
        // World to body
        Eigen::Matrix3d Rwb;
        Eigen::Vector3d Velocity;
        Eigen::Vector3d Position;

        Eigen::Vector3d GyroBias;
        Eigen::Vector3d AccelorometerBias;

        Eigen::Quaterniond q;
        Eigen::Vector3d t;

        typeNavigationState() = default;
        typeNavigationState(const Eigen::Quaterniond& Q, const Eigen::Vector3d& Vel, 
                const Eigen::Vector3d& Pos, const Eigen::Vector3d& GyroB, const Eigen::Vector3d& AccB) 
            : Rwb(Q.normalized().toRotationMatrix()),
            Velocity(Vel),
            Position(Pos),
            GyroBias(GyroB),
            AccelorometerBias(AccB),
            q(Q.normalized()),
            t(Position)
        {};

        typeNavigationState(const Eigen::Matrix3d& R, const Eigen::Vector3d& Vel, 
                const Eigen::Vector3d& Pos, const Eigen::Vector3d& GyroB, const Eigen::Vector3d& AccB) 
            : Rwb(R),
              Velocity(Vel),
              Position(Pos),
              GyroBias(GyroB),
              AccelorometerBias(AccB),
              q(R),
              t(Pos)
        {
            q.normalize();
        }

        void UpdateState(const typeNavigationState& NavigationState)
        {
            Rwb = NavigationState.Rwb;
            Velocity = NavigationState.Velocity;
            Position = NavigationState.Position;

            GyroBias = NavigationState.GyroBias;
            AccelorometerBias = NavigationState.AccelorometerBias;

            Eigen::Quaterniond qNew(Rwb);
            qNew.normalize();
            q = qNew;
            t = Position;
        }
};

class typePreIntegrationData
{
    public:
        Eigen::Matrix3d DeltaR;
        Eigen::Vector3d DeltaVelocity;
        Eigen::Vector3d DeltaPosition;
        fp64 DeltaT;

        Eigen::Vector3d GyroBias;
        Eigen::Vector3d AccelBias;

        Eigen::Matrix3d JRg;
        Eigen::Matrix3d JVg;
        Eigen::Matrix3d JVa;
        Eigen::Matrix3d JPg;
        Eigen::Matrix3d JPa;

        Eigen::Matrix<fp64, 12, 12> Qc;

        Eigen::Matrix<fp64, 15, 15> Covariance;


};

class typePreIntegration : public typePreIntegrationData
{
    public:
        typeNavigationState InitialNavigationState;
        typeIMUMeasurement PreviousMeasurement;
        bool HasPreviousMeasurement;

        typePreIntegration()
        {
            DeltaR = Eigen::Matrix3d::Identity();
            DeltaVelocity = {};
            DeltaPosition = {};
            DeltaT = 0.0;
            GyroBias = {};
            AccelBias = {};
            InitialNavigationState = {};
            PreviousMeasurement = {};
            HasPreviousMeasurement = false;

            Qc = {};
            Covariance = {};
        }

        void Reset(const typeNavigationState& NavigationState)
        {
            DeltaR.setIdentity();
            DeltaVelocity.setZero();
            DeltaPosition.setZero();
            DeltaT = 0.0;

            GyroBias = NavigationState.GyroBias;
            AccelBias = NavigationState.AccelorometerBias;
            InitialNavigationState = NavigationState;

            JRg.setZero();
            JVg.setZero();
            JVa.setZero();
            JPg.setZero();
            JPa.setZero();

            const typeIMUIntrinsics* IMUIntrinsics = IMU_GetIntrinsics();

            Qc.setZero();

            Qc.block<3,3>(0, 0) = IMUIntrinsics->GyroscopeNoiseDensity * IMUIntrinsics->GyroscopeNoiseDensity * Eigen::Matrix3d::Identity();
            Qc.block<3,3>(3, 3) = IMUIntrinsics->AccelerometerNoiseDensity * IMUIntrinsics->AccelerometerNoiseDensity * Eigen::Matrix3d::Identity();
            Qc.block<3,3>(6, 6) = IMUIntrinsics->GyroscopeRandomWalk * IMUIntrinsics->GyroscopeRandomWalk * Eigen::Matrix3d::Identity();
            Qc.block<3,3>(9, 9) = IMUIntrinsics->AccelerometerRandomWalk * IMUIntrinsics->AccelerometerRandomWalk * Eigen::Matrix3d::Identity();

            Covariance.setZero();
        }

        void UpdateJacobians( const Eigen::Matrix3d& dR, const Eigen::Matrix3d& Jr, const Eigen::Vector3d& Acc,
                fp64 dT)
        {
            const Eigen::Matrix3d AccSkew = Sophus::SO3d::hat(Acc);
            const fp64 dTSquared = dT * dT;

            JPa = JPa + JVa * dT - 0.5 * DeltaR * dTSquared;
            JPg = JPg + JVg * dT - 0.5 * DeltaR * AccSkew * JRg * dTSquared;

            JVa = JVa - DeltaR * dT;
            JVg = JVg - DeltaR * AccSkew * JRg * dT;

            JRg = dR.transpose() * JRg - Jr * dT;
        }

        void UpdateCovariance(const Eigen::Vector3d& Omega, const Eigen::Vector3d& Acc, fp64 dT)
        {
            using Matrix15 = Eigen::Matrix<fp64, 15, 15>;
            using Matrix15x12 = Eigen::Matrix<fp64, 15, 12>;

            Matrix15 F = Matrix15::Identity();
            Matrix15x12 G = Matrix15x12::Zero();

            const Eigen::Matrix3d OmegaSkew = Sophus::SO3d::hat(Omega);
            const Eigen::Matrix3d AccSkew = Sophus::SO3d::hat(Acc);
            const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

            F.block<3,3>(0,0) = I - OmegaSkew * dT;
            F.block<3,3>(0,9) = -I * dT;
            F.block<3,3>(3,0) = -DeltaR * AccSkew * dT;
            F.block<3,3>(3,12) = -DeltaR * dT;
            F.block<3,3>(6,0) =
                -0.5 * DeltaR * AccSkew * dT * dT;
            F.block<3,3>(6,3) = I * dT;
            F.block<3,3>(6,12) =
                -0.5 * DeltaR * dT * dT;

            G.block<3,3>(0,0) = -I;
            G.block<3,3>(3,3) = -DeltaR;
            G.block<3,3>(6,3) = -0.5 * DeltaR * dT;
            G.block<3,3>(9,6) = I;
            G.block<3,3>(12,9) = I;

            Covariance = F * Covariance * F.transpose() + G * Qc * G.transpose() * dT;
        }

        void PreIntegrate(const Eigen::Vector3d& Acc, const Eigen::Matrix3d& dR, fp64 dT)
        {
            DeltaPosition = DeltaPosition + 
                DeltaVelocity * dT + 
                0.5 * DeltaR * Acc * dT * dT;

            DeltaVelocity = DeltaVelocity + DeltaR * Acc * dT;

            DeltaR = DeltaR * dR;
            DeltaT += dT;
        }
};

void IMU_NewNavigationStateArrival(const typeNavigationState& NavigationState);
void IMU_InitializePreIntegration(typePreIntegration& PreIntegrationState,
        const typeNavigationState& NavigationState);
bool IMU_InitializeGravity(const typeNavigationState& NavigationState, const typeIMUMeasurement& Measurement, const Eigen::Vector3d& WorldAcceleration);
Eigen::Vector3d* IMU_GetGravity(void);
void IMU_IngegrationStep(const typeIMUMeasurement& Current);
void IMU_IngegrationStep(const typeIMUMeasurement& Current, typePreIntegration& PreIntegratioState);
void IMU_GetPreIntegratedRt(Eigen::Matrix3d& Rwb, Eigen::Vector3d& twb);
typePreIntegrationData IMU_GetLatestPreIntegrationData(void);
typeNavigationState IMU_PredictNavigationState(const typeNavigationState& PreviousNavigationState,
        const typePreIntegrationData& PreIntegrationData);

#endif //  IMU_PREINTEGRATION_HPP

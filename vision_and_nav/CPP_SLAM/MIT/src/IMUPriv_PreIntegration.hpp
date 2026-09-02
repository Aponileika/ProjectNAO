#ifndef IMUPRIV_PREINTEGRATION_HPP
#define IMUPRIV_PREINTEGRATION_HPP
#include "IMU_PreIntegration.hpp"

class typePreIntegration
{
    public:
        Eigen::Matrix3d DeltaR;
        Eigen::Vector3d DeltaVelocity;
        Eigen::Vector3d DeltaPosition;
        fp64 DeltaT;

        Eigen::Vector3d GyroBias;
        Eigen::Vector3d AccelBias;
        typeNavigationState InitialNavigationState;

        Eigen::Matrix3d JRg;
        Eigen::Matrix3d JVg;
        Eigen::Matrix3d JVa;
        Eigen::Matrix3d JPg;
        Eigen::Matrix3d JPa;

        Eigen::Matrix<fp64, 12, 12> Qc;

        Eigen::Matrix<fp64, 15, 15> Covariance;

        typePreIntegration()
        {
            DeltaR = Eigen::Matrix3d::Identity();
            DeltaVelocity = {};
            DeltaPosition = {};
            DeltaT = 0.0;
            GyroBias = {};
            AccelBias = {};
            InitialNavigationState = {};

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
            F.block<3,3>(6,3) = I * dT;

            G.block<3,3>(0,0) = -I;
            G.block<3,3>(3,3) = -DeltaR;
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

#endif  // IMUPRIV_PREINTEGRATION_HPP

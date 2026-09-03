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
            : Rwb(Q.toRotationMatrix()),
            Velocity(Vel),
            Position(Pos),
            GyroBias(GyroB),
            AccelorometerBias(AccB),
            q(Q),
            t(Position)
        {
            q.normalize();
        };

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

            Eigen::Quaterniond qNew(Rwb.transpose());
            qNew.normalize();
            q = qNew;
            t = -Rwb.transpose() * Position;
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

};

void IMU_NewNavigationStateArrival(const typeNavigationState& NavigationState);
bool IMU_InitializeGravity(const typeNavigationState& NavigationState,
        const typeIMUMeasurement& Measurement,
        const Eigen::Vector3d& WorldAcceleration);
void IMU_IngegrationStep(const typeIMUMeasurement& Current);
void IMU_GetPreIntegratedRt(Eigen::Matrix3d& Rwb, Eigen::Vector3d& twb);
typePreIntegrationData IMU_GetLatestPreIntegrationData(void);
typeNavigationState IMU_PredictNavigationState(const typeNavigationState& PreviousNavigationState,
        const typePreIntegrationData& PreIntegrationData);

#endif //  IMU_PREINTEGRATION_HPP

#ifndef IMU_PREINTEGRATION_HPP
#define IMU_PREINTEGRATION_HPP
#include <Eigen/Dense>
#include "CArenaAlloc.h"
#include "IMU_IMUReader.hpp"
#include "sophus/so3.hpp"
#include "CM_Camera.hpp"

typedef struct
{
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::Vector3d GyroBias;
    Eigen::Vector3d AccelorometerBias;
}typeNavigationState;

void IMU_NewNavigationStateArrival(const typeNavigationState& NavigationState);
void IMU_IngegrationStep(const typeIMUMeasurement& Current);
void IMU_GetPreIntegratedRt(Eigen::Matrix3d& Rwb, Eigen::Vector3d& twb);

#endif //  IMU_PREINTEGRATION_HPP

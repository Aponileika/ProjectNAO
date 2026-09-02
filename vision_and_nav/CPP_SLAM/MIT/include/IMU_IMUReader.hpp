/*For the moment this only reads csv format with:
 * timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
 * */
#ifndef IMU_IMUREADER_HPP_
#define IMU_IMUREADER_HPP_
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include <Eigen/Dense>

typedef struct
{
    u64 TimeStamp;

    Eigen::Vector3d AngularVelocity;
    Eigen::Vector3d Acceleration;
}typeIMUMeasurement;

typeIMUMeasurement IMU_GetMeasurement(void);

#endif // IMU_IMUREADER_HPP_

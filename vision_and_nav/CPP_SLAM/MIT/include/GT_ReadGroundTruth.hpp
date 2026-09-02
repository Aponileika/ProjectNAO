#ifndef GT_READGROUNDTRUTH_HPP
#define GT_READGROUNDTRUTH_HPP
#include "Config.hpp"
#include <Eigen/Dense>
#include <vector>

typedef struct
{
    fp64 TimeStamp;   // [s]

    Eigen::Vector3d Position;        // p_RS_R [m]
    Eigen::Quaterniond Orientation;  // q_RS, stored as (w, x, y, z)
    Eigen::Vector3d Velocity;        // v_RS_R [m/s]

    Eigen::Vector3d GyroBias;        // b_w_RS_S [rad/s]
    Eigen::Vector3d AccelBias;       // b_a_RS_S [m/s^2]

} typeGroundTruth;

typeGroundTruth GT_GetMeasurement(void);
std::vector<typeGroundTruth> GT_GetAllMeasurements(void);

#endif // GT_READGROUNDTRUTH_HPP

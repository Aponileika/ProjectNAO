#ifndef DENSE_DENSEMAPPING_HPP
#define DENSE_DENSEMAPPING_HPP
#include "FR_Frames.hpp"
#include <Eigen/Dense>

typedef struct
{
    u64 KeyFrameID;
    Eigen::Matrix<fp64, 3, Eigen::Dynamic> CameraPoints; // Points in ccs
}typeDenseKeyFrameMap;

#endif // DENSE_DENSEMAPPING_HPP

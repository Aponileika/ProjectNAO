#ifndef GT_ALIGNMENT_HPP_
#define GT_ALIGNMENT_HPP_

#include "GT_ReadGroundTruth.hpp"
#include <Eigen/Geometry>

struct typeGTTrajectoryPose
{
    fp64 TimeStamp;
    Eigen::Vector3d Position;
    Eigen::Quaterniond Quaternion; // camera to world
};

struct typeGTAlignmentResult
{
    Eigen::Matrix3d Rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Translation = Eigen::Vector3d::Zero();
    std::size_t NumMatchedPoses = 0;
    std::size_t NumRelativePairs = 0;
    fp64 MeanTranslationError = 0.0; // meters, arithmetic mean of distances
    fp64 MeanAngularError = 0.0; // radians, aligned absolute SO(3) distance
    fp64 MeanRelativeRotationError = 0.0; // radians, consecutive matched poses
};

// Rigid least-squares position fit, with unit scale and det(R) = +1.
// Interpolates GT body poses at camera timestamps, then applies T_BS (camera
// to body). Does not extrapolate. Needs three non-collinear matched positions.
bool GT_ComputeTrajectoryAlignment(const std::vector<typeGTTrajectoryPose>& Trajectory,
        const std::vector<typeGroundTruth>& GroundTruth, const Eigen::Matrix4d& TBS,
        typeGTAlignmentResult& Result);

#endif

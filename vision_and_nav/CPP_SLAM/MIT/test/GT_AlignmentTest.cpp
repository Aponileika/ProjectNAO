#include "GT_Alignment.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "../src/GT_Alignment.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

static Eigen::Quaterniond Rotation(fp64 Angle)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(Angle, Eigen::Vector3d(1, 2, -1).normalized()));
}

static void TestFitAndInterpolation()
{
    const std::vector<Eigen::Vector3d> BodyPositions{{0,0,0}, {1,0,0}, {1,1,0}, {1,1,1}, {2,1,1}};
    std::vector<typeGroundTruth> GroundTruth;
    for(std::size_t i = 0; i < BodyPositions.size(); ++i)
    {
        typeGroundTruth Sample{};
        Sample.TimeStamp = static_cast<fp64>(i);
        Sample.Position = BodyPositions[i];
        Sample.Orientation = Rotation(0.2 * i);
        if(i % 2 == 1)
        {
            Sample.Orientation.coeffs() *= -1.0; // same SO(3) element
        }
        GroundTruth.push_back(Sample);
    }
    Eigen::Matrix4d TBS = Eigen::Matrix4d::Identity();
    TBS.block<3,3>(0,0) = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    TBS.block<3,1>(0,3) = Eigen::Vector3d(0.1, -0.2, 0.3);
    const Eigen::Quaterniond AlignmentRotation = Rotation(0.7);
    const Eigen::Vector3d AlignmentTranslation(3, -2, 1);
    std::vector<typeGTTrajectoryPose> Estimates;
    for(std::size_t i = 0; i <= 8; ++i)
    {
        const fp64 Time = 0.5 * i;
        const std::size_t Index = i / 2;
        const Eigen::Vector3d Position = i % 2 == 0 ? BodyPositions[Index] :
            (0.5 * (BodyPositions[Index] + BodyPositions[Index + 1])).eval();
        const Eigen::Quaterniond BodyRotation = Rotation(0.2 * Time);
        const Eigen::Vector3d CameraPosition = Position + BodyRotation * TBS.block<3,1>(0,3);
        const Eigen::Quaterniond CameraRotation = BodyRotation * Eigen::Quaterniond(TBS.block<3,3>(0,0));
        Estimates.push_back({Time, AlignmentRotation.conjugate() * (CameraPosition - AlignmentTranslation),
                AlignmentRotation.conjugate() * CameraRotation});
    }
    typeGTAlignmentResult Result;
    Require(GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, TBS, Result), "SE3 fit with interpolated GT");
    Require(Result.Rotation.isApprox(AlignmentRotation.toRotationMatrix(), 1e-10), "known rigid rotation recovered");
    Require(Result.Translation.isApprox(AlignmentTranslation, 1e-10), "known rigid translation recovered");
    Require(Result.MeanTranslationError < 1e-10 && Result.MeanAngularError < 1e-10 &&
            Result.MeanRelativeRotationError < 1e-10, "aligned pose and relative errors vanish");
    Require(Result.NumMatchedPoses == 9 && Result.NumRelativePairs == 8, "all timestamps including endpoints associated");

    const Eigen::Quaterniond WorldError(Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()));
    for(typeGTTrajectoryPose& Estimate : Estimates)
    {
        Estimate.Quaternion = AlignmentRotation.conjugate() * WorldError * AlignmentRotation * Estimate.Quaternion;
    }
    Require(GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, TBS, Result), "orientation-error fixture");
    Require(std::abs(Result.MeanAngularError - 0.12) < 1e-10, "mean SO3 error is the geodesic angle");
    Require(Result.MeanRelativeRotationError < 1e-10, "relative error excludes constant world rotation offset");

    Estimates.insert(Estimates.begin(), {-1.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()});
    Estimates.push_back({5.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()});
    Estimates[4].Position.x() = std::numeric_limits<fp64>::quiet_NaN();
    Require(GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, TBS, Result), "invalid and out-of-range poses skipped");
    Require(Result.NumMatchedPoses == 8 && Result.NumRelativePairs == 6, "no extrapolation or relative pairs across invalid poses");
}

static void TestScaleMeanAndRelativeRotation()
{
    const std::vector<Eigen::Vector3d> Positions{{1,0,0}, {-1,0,0}, {0,2,0}, {0,-2,0}, {0,0,3}, {0,0,-3}};
    std::vector<typeGroundTruth> GroundTruth;
    std::vector<typeGTTrajectoryPose> Estimates;
    for(std::size_t i = 0; i < Positions.size(); ++i)
    {
        typeGroundTruth Sample{};
        Sample.TimeStamp = static_cast<fp64>(i);
        Sample.Position = Positions[i];
        Sample.Orientation = Eigen::Quaterniond::Identity();
        GroundTruth.push_back(Sample);
        Estimates.push_back({Sample.TimeStamp, 2.0 * Positions[i],
                Eigen::Quaterniond(Eigen::AngleAxisd(0.05 * i, Eigen::Vector3d::UnitZ()))});
    }
    typeGTAlignmentResult Result;
    Require(GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, Eigen::Matrix4d::Identity(), Result), "scale error fixture");
    Require(std::abs(Result.MeanTranslationError - 2.0) < 1e-10, "ATE is mean distance, not RMSE, and scale is fixed");
    Require(std::abs(Result.MeanRelativeRotationError - 0.05) < 1e-10, "consecutive-pose RRE");
    Require(std::abs(Result.MeanAngularError - 0.125) < 1e-10, "absolute SO3 arithmetic mean");
    Require(std::abs(Result.Rotation.determinant() - 1.0) < 1e-10, "proper rotation");

    for(typeGTTrajectoryPose& Estimate : Estimates)
    {
        Estimate.Position.x() *= -1.0;
    }
    Require(GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, Eigen::Matrix4d::Identity(), Result), "reflection fixture");
    Require(Result.Rotation.determinant() > 0.999999, "fit does not introduce a reflection");
    Require(Result.MeanTranslationError > 0.1, "reflection cannot fit exactly");

    for(std::size_t i = 0; i < GroundTruth.size(); ++i)
    {
        GroundTruth[i].Position = Eigen::Vector3d(static_cast<fp64>(i), 0, 0);
        Estimates[i].Position = GroundTruth[i].Position;
    }
    Require(!GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, Eigen::Matrix4d::Identity(), Result), "collinear fit rejected");
    GroundTruth.clear();
    Require(!GT_ComputeTrajectoryAlignment(Estimates, GroundTruth, Eigen::Matrix4d::Identity(), Result), "missing GT handled");
}

int main()
{
    TestFitAndInterpolation();
    TestScaleMeanAndRelativeRotation();
    std::cout << "Ground-truth SE3 alignment tests passed\n";
}

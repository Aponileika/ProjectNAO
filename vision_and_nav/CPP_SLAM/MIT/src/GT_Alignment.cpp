#include "GT_Alignment.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

static fp64 GTPriv_RotationDistance(const Eigen::Quaterniond& First, const Eigen::Quaterniond& Second)
{
    const Eigen::Quaterniond Difference = (First.conjugate() * Second).normalized();
    return 2.0 * std::atan2(Difference.vec().norm(), std::abs(Difference.w()));
}

bool GT_ComputeTrajectoryAlignment(const std::vector<typeGTTrajectoryPose>& Trajectory,
        const std::vector<typeGroundTruth>& GroundTruth, const Eigen::Matrix4d& TBS,
        typeGTAlignmentResult& Result)
{
    Result = {};
    if(GroundTruth.empty() || !TBS.allFinite() || !TBS.block<3,3>(0,0).isUnitary(1e-6) ||
       TBS.block<3,3>(0,0).determinant() < 0.0)
    {
        return false;
    }
    for(std::size_t i = 0; i < GroundTruth.size(); ++i)
    {
        const typeGroundTruth& Sample = GroundTruth[i];
        if(!std::isfinite(Sample.TimeStamp) || !Sample.Position.allFinite() ||
           !Sample.Orientation.coeffs().allFinite() || !std::isfinite(Sample.Orientation.norm()) ||
           Sample.Orientation.norm() < 1e-12 ||
           (i > 0 && Sample.TimeStamp <= GroundTruth[i - 1].TimeStamp))
        {
            return false;
        }
    }

    struct typeMatchedPose
    {
        std::size_t Index;
        typeGTTrajectoryPose Estimate;
        typeGTTrajectoryPose Reference;
    };
    std::vector<typeMatchedPose> Matched;
    Matched.reserve(Trajectory.size());
    const Eigen::Quaterniond CameraToBody(TBS.block<3,3>(0,0));
    const Eigen::Vector3d CameraInBody = TBS.block<3,1>(0,3);

    for(std::size_t i = 0; i < Trajectory.size(); ++i)
    {
        const typeGTTrajectoryPose& Estimate = Trajectory[i];
        if(!std::isfinite(Estimate.TimeStamp) || !Estimate.Position.allFinite() ||
           !Estimate.Quaternion.coeffs().allFinite() || !std::isfinite(Estimate.Quaternion.norm()) ||
           Estimate.Quaternion.norm() < 1e-12 ||
           Estimate.TimeStamp < GroundTruth.front().TimeStamp || Estimate.TimeStamp > GroundTruth.back().TimeStamp)
        {
            continue;
        }
        const auto Next = std::lower_bound(GroundTruth.begin(), GroundTruth.end(), Estimate.TimeStamp,
                [](const typeGroundTruth& Sample, fp64 TimeStamp) { return Sample.TimeStamp < TimeStamp; });
        Eigen::Vector3d BodyPosition = Next->Position;
        Eigen::Quaterniond BodyRotation = Next->Orientation.normalized();
        if(Next != GroundTruth.begin() && Next->TimeStamp > Estimate.TimeStamp)
        {
            const typeGroundTruth& Previous = *(Next - 1);
            const fp64 Alpha = (Estimate.TimeStamp - Previous.TimeStamp) / (Next->TimeStamp - Previous.TimeStamp);
            BodyPosition = (1.0 - Alpha) * Previous.Position + Alpha * Next->Position;
            BodyRotation = Previous.Orientation.normalized().slerp(Alpha, BodyRotation).normalized();
        }
        Matched.push_back({i, Estimate, {Estimate.TimeStamp,
                    BodyPosition + BodyRotation * CameraInBody, (BodyRotation * CameraToBody).normalized()}});
    }
    Result.NumMatchedPoses = Matched.size();
    if(Matched.size() < 3)
    {
        return false;
    }

    Eigen::Vector3d EstimateCenter = Eigen::Vector3d::Zero();
    Eigen::Vector3d ReferenceCenter = Eigen::Vector3d::Zero();
    for(const typeMatchedPose& Pose : Matched)
    {
        EstimateCenter += Pose.Estimate.Position;
        ReferenceCenter += Pose.Reference.Position;
    }
    EstimateCenter /= static_cast<fp64>(Matched.size());
    ReferenceCenter /= static_cast<fp64>(Matched.size());
    Eigen::Matrix3d CrossCovariance = Eigen::Matrix3d::Zero();
    for(const typeMatchedPose& Pose : Matched)
    {
        CrossCovariance += (Pose.Reference.Position - ReferenceCenter) * (Pose.Estimate.Position - EstimateCenter).transpose();
    }
    if(!CrossCovariance.allFinite())
    {
        return false;
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> SVD(CrossCovariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(SVD.singularValues()(0) < 1e-12 || SVD.singularValues()(1) < 1e-10 * SVD.singularValues()(0))
    {
        return false; // Position-only alignment cannot resolve a line's twist.
    }
    Eigen::Matrix3d Sign = Eigen::Matrix3d::Identity();
    Sign(2,2) = (SVD.matrixU() * SVD.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    Result.Rotation = SVD.matrixU() * Sign * SVD.matrixV().transpose();
    Result.Translation = ReferenceCenter - Result.Rotation * EstimateCenter;
    const Eigen::Quaterniond AlignmentRotation(Result.Rotation);

    for(std::size_t i = 0; i < Matched.size(); ++i)
    {
        const typeMatchedPose& Pose = Matched[i];
        Result.MeanTranslationError +=
            (Result.Rotation * Pose.Estimate.Position + Result.Translation - Pose.Reference.Position).norm();
        Result.MeanAngularError += GTPriv_RotationDistance(Pose.Reference.Quaternion,
                AlignmentRotation * Pose.Estimate.Quaternion.normalized());
        if(i > 0 && Pose.Index == Matched[i - 1].Index + 1 &&
           Pose.Estimate.TimeStamp > Matched[i - 1].Estimate.TimeStamp)
        {
            const typeMatchedPose& Previous = Matched[i - 1];
            const Eigen::Quaterniond EstimatedRelative = Previous.Estimate.Quaternion.normalized().conjugate() *
                Pose.Estimate.Quaternion.normalized();
            const Eigen::Quaterniond ReferenceRelative = Previous.Reference.Quaternion.conjugate() * Pose.Reference.Quaternion;
            Result.MeanRelativeRotationError += GTPriv_RotationDistance(ReferenceRelative, EstimatedRelative);
            ++Result.NumRelativePairs;
        }
    }
    Result.MeanTranslationError /= static_cast<fp64>(Matched.size());
    Result.MeanAngularError /= static_cast<fp64>(Matched.size());
    if(Result.NumRelativePairs > 0)
    {
        Result.MeanRelativeRotationError /= static_cast<fp64>(Result.NumRelativePairs);
    }
    return Result.Rotation.allFinite() && Result.Translation.allFinite() &&
        std::isfinite(Result.MeanTranslationError) && std::isfinite(Result.MeanAngularError) &&
        std::isfinite(Result.MeanRelativeRotationError);
}

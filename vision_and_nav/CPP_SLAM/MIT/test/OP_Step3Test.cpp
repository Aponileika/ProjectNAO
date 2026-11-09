#include "OP_BA.hpp"
#include <array>
#include <cstdlib>
#include <iostream>

// Like the other test executable, use the real camera/navigation helpers.
#include "../src/LG_Logging.cpp"
#include "../src/CM_Camera.cpp"
#include "../src/KEY_KeyFrame.cpp"
#include "../src/OP_BA.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

struct Step3Fixture
{
    static constexpr std::size_t Count = 5;
    const std::array<u64, Count> IDs{41, 7, 99, 20, 3};
    typeCameraIntrinsics Intrinsics = *CM_GetIntrinsics();
    typePantoVector<typeKeyFrame> KeyFrames;
    typePantoVector<typePantoMapPoint> MapPoints;
    std::array<Eigen::Matrix3d, Count> BodyRotations;
    std::array<Eigen::Vector3d, Count> TrueCenters;

    typeKeyFrame& Frame(std::size_t TemporalIndex)
    {
        for(typeKeyFrame& KeyFrame : KeyFrames)
        {
            if(KeyFrame.ID == IDs[TemporalIndex])
            {
                return KeyFrame;
            }
        }
        std::abort();
    }

    explicit Step3Fixture(bool Stereo)
    {
        Intrinsics.T_BS.setIdentity();
        Intrinsics.T_BS.block<3,3>(0,0) = Sophus::SO3d::exp(Eigen::Vector3d(0.13, -0.21, 0.09)).matrix();
        Intrinsics.T_BS.block<3,1>(0,3) = Eigen::Vector3d(0.07, -0.04, 0.12);
        const Eigen::Matrix3d Rbc = Intrinsics.T_BS.block<3,3>(0,0);
        BodyRotations[0] = Sophus::SO3d::exp(Eigen::Vector3d(0.1, 0.05, -0.15)).matrix();
        for(std::size_t i = 0; i < Count; ++i)
        {
            if(i != 0)
            {
                BodyRotations[i] = BodyRotations[i - 1] *
                    Sophus::SO3d::exp(Eigen::Vector3d(0.025 * i, -0.012, 0.018)).matrix();
            }
            TrueCenters[i] = Eigen::Vector3d(0.12 * i, -0.04 * i, 0.02 * i);
        }

        for(u64 i = 0; i < 36; ++i)
        {
            const Eigen::Vector3d LocalPoint(-1.0 + (i % 6) * 0.4, -0.8 + (i / 6) * 0.3, 4.0 + 0.15 * (i % 5));
            const Eigen::Vector3d WorldPoint = BodyRotations[0] * Rbc * LocalPoint;
            typePantoMapPoint MapPoint{};
            MapPoint.ID = i;
            MapPoint.Point << WorldPoint, 1.0;
            // Exercise homogeneous coordinates, including negative scaling.
            MapPoint.Point *= (i % 2 == 0 ? -2.0 : 0.3);
            MapPoints.push_back(MapPoint);
        }

        // A removed slot and nonchronological insertion reproduce a sliding
        // window backed by recycled typePantoVector storage.
        KeyFrames.push_back(typeKeyFrame{});
        KeyFrames.push_back(typeKeyFrame{});
        KeyFrames.push_back(typeKeyFrame{});
        KeyFrames.remove(0);
        KeyFrames.remove(2);
        for(const std::size_t i : {2, 0, 4, 1, 3})
        {
            typeKeyFrame KeyFrame{};
            KeyFrame.ID = IDs[i];
            KeyFrame.PreviousKFID = (i == 0 ? 500 : IDs[i - 1]);
            KeyFrame.NextKFID = (i + 1 == Count ? PANTO_ID_NOT_SET : IDs[i + 1]);
            KeyFrame.Camera.Intrinsics = &Intrinsics;
            const Eigen::Matrix3d Rwc = BodyRotations[i] * Rbc;
            const Eigen::Matrix3d VisualRwc = Rwc * Sophus::SO3d::exp(
                    Eigen::Vector3d(0.015 * i, -0.01 * i, 0.02 * i)).matrix();
            KeyFrame.Camera.Pose.Quaternion = Eigen::Quaterniond(VisualRwc);
            KeyFrame.Camera.Pose.tParametrization = TrueCenters[i] + Eigen::Vector3d(0.08, -0.06, 0.1);
            KeyFrame.Camera.Pose.UpdateRt();
            KeyFrame.NavigationState = typeNavigationState(Eigen::Matrix3d::Identity().eval(),
                    Eigen::Vector3d(0.2, -0.1, 0.3), Eigen::Vector3d::Zero().eval(),
                    Eigen::Vector3d(0.01 + 0.002 * i, -0.02, 0.005), Eigen::Vector3d(0.1, 0.02, -0.03));
            KEY_UpdateNavState(&KeyFrame);
            if(i != 0)
            {
                typePreIntegrationData& PreIntegration = KeyFrame.PreIntegrationData;
                PreIntegration.DeltaT = 0.1;
                PreIntegration.GyroBias = Eigen::Vector3d(-0.004, 0.006 * i, -0.003);
                PreIntegration.JRg = -0.1 * Eigen::Matrix3d::Identity();
                PreIntegration.JRg(0, 1) = 0.004;
                const Eigen::Vector3d SourceBias(0.01 + 0.002 * (i - 1), -0.02, 0.005);
                const Eigen::Vector3d Correction = PreIntegration.JRg * (SourceBias - PreIntegration.GyroBias);
                PreIntegration.DeltaR = BodyRotations[i - 1].transpose() * BodyRotations[i] *
                    Sophus::SO3d::exp(-Correction).matrix();
            }

            for(const typePantoMapPoint& MapPoint : MapPoints)
            {
                const Eigen::Vector3d Xc = Rwc.transpose() *
                    (MapPoint.Point.head<3>() / MapPoint.Point.w() - TrueCenters[i]);
                typePantoImagePoint ImagePoint{};
                ImagePoint.ID = KeyFrame.Points.ImagePoints.active_size();
                ImagePoint.MapPointID = MapPoint.ID;
                ImagePoint.Point = Eigen::Vector2d(OPCameraIntrinsics.fx * Xc.x() / Xc.z() + OPCameraIntrinsics.cx,
                        OPCameraIntrinsics.fy * Xc.y() / Xc.z() + OPCameraIntrinsics.cy);
                ImagePoint.IsMatched = Stereo && MapPoint.ID % 2 == 0;
                ImagePoint.RightCameraMatch = Eigen::Vector2d(
                        OPStereoIntrinsics.fx * (Xc.x() - OPStereoIntrinsics.B) / Xc.z() + OPStereoIntrinsics.cx,
                        ImagePoint.Point.y());
                KeyFrame.Points.ImagePoints.push_back(ImagePoint);
            }
            // Out-of-range associations must be skipped before indexing.
            typePantoImagePoint MissingPoint{};
            MissingPoint.MapPointID = 10000;
            KeyFrame.Points.ImagePoints.push_back(MissingPoint);
            KeyFrames.push_back(KeyFrame);
        }
        KeyFrames.remove(1);
    }
};

static void TestRecovery(bool Stereo)
{
    Step3Fixture Fixture(Stereo);
    const auto OriginalPoints = Fixture.MapPoints;
    const auto OriginalPreIntegration = Fixture.Frame(1).PreIntegrationData;
    Require(OP_UpdateRotationsFromPreIntegration(Fixture.KeyFrames), "rotation update");
    for(std::size_t i = 0; i < Fixture.Count; ++i)
    {
        const typeKeyFrame& KeyFrame = Fixture.Frame(i);
        Require(KeyFrame.NavigationState.Rwb.isApprox(Fixture.BodyRotations[i], 1e-12), "bias-corrected body rotation");
        Require(KeyFrame.Camera.Pose.tParametrization.isApprox(
                    Fixture.TrueCenters[i] + Eigen::Vector3d(0.08, -0.06, 0.1), 1e-12), "rotation update preserves center");
    }
    const auto RotatedFrames = Fixture.KeyFrames;
    Require(OP_BundleAdjustTranslationOnly(Fixture.KeyFrames, Fixture.MapPoints), "translation solve");
    for(std::size_t i = 0; i < Fixture.Count; ++i)
    {
        const typeKeyFrame& KeyFrame = Fixture.Frame(i);
        Require((KeyFrame.Camera.Pose.tParametrization - Fixture.TrueCenters[i]).norm() < 1e-6, "metric translation recovery");
        Require(KeyFrame.NavigationState.Rwb.isApprox(Fixture.BodyRotations[i], 1e-12), "translation solve preserves rotation");
        Require(KeyFrame.NavigationState.Velocity == Eigen::Vector3d(0.2, -0.1, 0.3), "velocity preserved");
        Require(KeyFrame.NavigationState.AccelorometerBias == Eigen::Vector3d(0.1, 0.02, -0.03), "accel bias preserved");
        Require(KeyFrame.NavigationState.GyroBias == Eigen::Vector3d(0.01 + 0.002 * i, -0.02, 0.005), "gyro bias preserved");
        const Eigen::Vector3d BodyPosition = Fixture.TrueCenters[i] -
            Fixture.BodyRotations[i] * Fixture.Intrinsics.T_BS.block<3,1>(0,3);
        Require((KeyFrame.NavigationState.Position - BodyPosition).norm() < 1e-6, "camera/body lever arm");
        Require(KeyFrame.NavigationState.t == KeyFrame.NavigationState.Position, "navigation translation synchronized");
        Require(KeyFrame.NavigationState.q.toRotationMatrix().isApprox(Fixture.BodyRotations[i], 1e-12), "navigation quaternion synchronized");
        Require(KeyFrame.Camera.Pose.R.isApprox(KeyFrame.Camera.Pose.Quaternion.toRotationMatrix().transpose(), 1e-12), "camera rotation synchronized");
        Require((KeyFrame.Camera.Pose.t + KeyFrame.Camera.Pose.R * Fixture.TrueCenters[i]).norm() < 1e-6, "camera extrinsic translation synchronized");
    }
    for(auto It = Fixture.KeyFrames.begin(); It != Fixture.KeyFrames.end(); ++It)
    {
        Require(It->Camera.Pose.Quaternion.coeffs() == RotatedFrames[It.index()].Camera.Pose.Quaternion.coeffs(), "fixed quaternion coefficients");
    }
    for(std::size_t i = 0; i < Fixture.MapPoints.size(); ++i)
    {
        Require(Fixture.MapPoints[i].Point == OriginalPoints[i].Point, "fixed map point");
    }
    Require(Fixture.Frame(1).PreIntegrationData.DeltaR == OriginalPreIntegration.DeltaR, "nominal preintegration preserved");
    Require(Fixture.Frame(1).PreIntegrationData.GyroBias == OriginalPreIntegration.GyroBias, "linearization bias preserved");
}

static void TestFailureAndCombinedCall()
{
    Step3Fixture Fixture(true);
    auto OriginalFrames = Fixture.KeyFrames;
    const Eigen::Quaterniond OriginalRotation = Fixture.Frame(1).Camera.Pose.Quaternion;
    Fixture.Frame(3).PreIntegrationData.DeltaT = 0.0;
    Require(!OP_UpdateRotationsFromPreIntegration(Fixture.KeyFrames), "invalid interval rejected");
    Require(Fixture.Frame(1).Camera.Pose.Quaternion.coeffs() == OriginalRotation.coeffs(), "invalid interval leaves earlier rotation unchanged");
    Fixture.KeyFrames = OriginalFrames;
    Fixture.Frame(2).PreviousKFID = 12345;
    Require(!OP_UpdateRotationsFromPreIntegration(Fixture.KeyFrames), "disconnected window rejected");
    Fixture.KeyFrames = OriginalFrames;
    Fixture.Frame(0).PreviousKFID = Fixture.IDs.back();
    Require(!OP_UpdateRotationsFromPreIntegration(Fixture.KeyFrames), "cycle rejected");
    Fixture.KeyFrames = OriginalFrames;
    Fixture.Frame(4).Points.ImagePoints.clear();
    OriginalFrames = Fixture.KeyFrames;
    Require(!OP_RotationTranslationDecoupledOptimization(Fixture.KeyFrames, Fixture.MapPoints), "unsupported translation rejected");
    for(auto It = Fixture.KeyFrames.begin(); It != Fixture.KeyFrames.end(); ++It)
    {
        const typeKeyFrame& Current = *It;
        const typeKeyFrame& Original = OriginalFrames[It.index()];
        Require(Current.Camera.Pose.Quaternion.coeffs() == Original.Camera.Pose.Quaternion.coeffs(), "combined call rolls back rotation");
        Require(Current.Camera.Pose.tParametrization == Original.Camera.Pose.tParametrization, "combined call rolls back center");
        Require(Current.NavigationState.Rwb == Original.NavigationState.Rwb, "combined call rolls back body rotation");
        Require(Current.NavigationState.Position == Original.NavigationState.Position, "combined call rolls back body position");
    }

    Step3Fixture ValidFixture(true);
    Require(OP_RotationTranslationDecoupledOptimization(ValidFixture.KeyFrames, ValidFixture.MapPoints), "combined Step 3 call");
    for(std::size_t i = 0; i < Fixture.Count; ++i)
    {
        Require((ValidFixture.Frame(i).Camera.Pose.tParametrization - ValidFixture.TrueCenters[i]).norm() < 1e-6,
                "combined Step 3 translation recovery");
    }
}

static void TestOutlier()
{
    Step3Fixture Fixture(true);
    Require(OP_UpdateRotationsFromPreIntegration(Fixture.KeyFrames), "outlier fixture rotation update");
    Fixture.Frame(1).Points.ImagePoints[0].Point += Eigen::Vector2d(200.0, -150.0);
    Require(OP_BundleAdjustTranslationOnly(Fixture.KeyFrames, Fixture.MapPoints), "robust translation solve");
    Require((Fixture.Frame(1).Camera.Pose.tParametrization - Fixture.TrueCenters[1]).norm() < 0.005,
            "Huber loss limits gross outlier influence");
}

int main()
{
    TestRecovery(false);
    TestRecovery(true);
    TestFailureAndCombinedCall();
    TestOutlier();
    std::cout << "Stereo-NEC Step 3 tests passed\n";
}

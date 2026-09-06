#include "IMU_PreIntegration.hpp"
#include "IMUPriv_PreIntegration.hpp"

typePreIntegration IntegrationState = typePreIntegration(); 

static Eigen::Vector3d g{};
static Eigen::Vector3d GravityDirectionSum{};
static std::size_t GravityInitializationSamples{};

void IMU_NewNavigationStateArrival(const typeNavigationState& NavigationState)
{
    IntegrationState.Reset(NavigationState);
}

void IMU_InitializePreIntegration(typePreIntegration& PreIntegrationState,
        const typeNavigationState& NavigationState)
{
    PreIntegrationState.PreviousMeasurement = IntegrationState.PreviousMeasurement;
    PreIntegrationState.HasPreviousMeasurement = IntegrationState.HasPreviousMeasurement;
    PreIntegrationState.Reset(NavigationState);
}

void IMU_ResetGravityInitialization(void)
{
    g.setZero();
    GravityDirectionSum.setZero();
    GravityInitializationSamples = 0;
}

bool IMU_AddGravityInitializationMeasurement(
        const typeNavigationState& NavigationState,
        const typeIMUMeasurement& Measurement,
        const Eigen::Vector3d& WorldAcceleration)
{
    const Eigen::Vector3d SpecificForce =
        Measurement.Acceleration - NavigationState.AccelorometerBias;

    // Accelerometers measure specific force:
    //     f_B = R_BW * (a_W - g_W)
    // Therefore, with the GT body-to-world rotation and GT acceleration:
    //     g_W = a_W - R_WB * f_B
    const Eigen::Vector3d GravityEstimate =
        WorldAcceleration - NavigationState.Rwb * SpecificForce;

    if(!GravityEstimate.allFinite() || GravityEstimate.norm() < 1e-6)
    {
        return false;
    }

    // Average directions rather than raw vectors so that a noisy numerical GT
    // acceleration sample cannot dominate the complete initialization window.
    GravityDirectionSum += GravityEstimate.normalized();
    GravityInitializationSamples++;

    return true;
}

bool IMU_FinalizeGravityInitialization(void)
{
    constexpr fp64 GravityMagnitude = 9.81;

    if(GravityInitializationSamples == 0 ||
       !GravityDirectionSum.allFinite() ||
       GravityDirectionSum.norm() < 1e-6)
    {
        return false;
    }

    g = GravityMagnitude * GravityDirectionSum.normalized();
    const fp64 MeanDirectionAgreement =
        GravityDirectionSum.norm() /
        static_cast<fp64>(GravityInitializationSamples);

    LG_Log(LogSeverity::DATA,
            "[IMUGravityInitialization] samples = %zu; direction agreement = %.6f; g_W = (%.6f, %.6f, %.6f), |g| = %.6f\n",
            GravityInitializationSamples,
            MeanDirectionAgreement,
            g.x(), g.y(), g.z(), g.norm());

    return true;
}

Eigen::Vector3d* IMU_GetGravity(void)
{
    return &g;
}

static void IMUPriv_IntegrationStep(const typeIMUMeasurement& Current,
        typePreIntegration& PreIntegrationState)
{
    if(!PreIntegrationState.HasPreviousMeasurement)
    {
        PreIntegrationState.PreviousMeasurement = Current;
        PreIntegrationState.HasPreviousMeasurement = true;
        return;
    }

    const fp64 dT = Current.TimeStamp -
        PreIntegrationState.PreviousMeasurement.TimeStamp;

    if(dT <= 0.0)
    {
        PreIntegrationState.PreviousMeasurement = Current;
        return;
    }

    // Remove bias
    const Eigen::Vector3d Omega = Current.AngularVelocity - PreIntegrationState.GyroBias;
    const Eigen::Vector3d Acc = Current.Acceleration - PreIntegrationState.AccelBias;

    const Eigen::Vector3d Phi = Omega*dT;
    const Eigen::Matrix3d dR = Sophus::SO3d::exp(Phi).matrix();

    const Eigen::Matrix3d Jr = Sophus::SO3d::leftJacobian(-Phi);

    PreIntegrationState.UpdateJacobians(dR, Jr, Acc, dT);
    PreIntegrationState.UpdateCovariance(Omega, Acc, dT);
    PreIntegrationState.PreIntegrate(Acc, dR, dT);

    PreIntegrationState.PreviousMeasurement = Current;
}

void IMU_IngegrationStep(const typeIMUMeasurement& Current)
{
    IMUPriv_IntegrationStep(Current, IntegrationState);
}

void IMU_IngegrationStep(const typeIMUMeasurement& Current,
        typePreIntegration& PreIntegrationState)
{
    IMUPriv_IntegrationStep(Current, PreIntegrationState);
}

void IMU_GetPreIntegratedRt(Eigen::Matrix3d& Rwb, Eigen::Vector3d& twb)
{
    const typeNavigationState& InitialState =
        IntegrationState.InitialNavigationState;
    const typeNavigationState Prediction =
        IMU_PredictNavigationState(InitialState, IntegrationState);

    Rwb = Prediction.Rwb;
    twb = Prediction.Position;
}

typePreIntegrationData IMU_GetLatestPreIntegrationData(void)
{
    return IntegrationState;
}

typeNavigationState IMU_PredictNavigationState(const typeNavigationState& PreviousNavigationState,
        const typePreIntegrationData& PreIntegrationData)
{
    const Eigen::Vector3d DeltaGyroBias = PreviousNavigationState.GyroBias - PreIntegrationData.GyroBias;
    const Eigen::Vector3d DeltaAccelBias = PreviousNavigationState.AccelorometerBias - PreIntegrationData.AccelBias;

    const Sophus::SO3d DeltaRNominal(PreIntegrationData.DeltaR);
    const Eigen::Matrix3d DeltaRCorrected = (DeltaRNominal * Sophus::SO3d::exp(
            PreIntegrationData.JRg * DeltaGyroBias)).matrix();

    const Eigen::Vector3d DeltaVelocityCorrected = PreIntegrationData.DeltaVelocity + PreIntegrationData.JVg * DeltaGyroBias +
        PreIntegrationData.JVa * DeltaAccelBias;

    const Eigen::Vector3d DeltaPositionCorrected = PreIntegrationData.DeltaPosition + PreIntegrationData.JPg * DeltaGyroBias +
        PreIntegrationData.JPa * DeltaAccelBias;

    const Eigen::Matrix3d PredictedR = PreviousNavigationState.Rwb * DeltaRCorrected;
    const Eigen::Vector3d PredictedVelocity = PreviousNavigationState.Velocity +
        g * PreIntegrationData.DeltaT + PreviousNavigationState.Rwb * DeltaVelocityCorrected;

    const Eigen::Vector3d PredictedPosition = PreviousNavigationState.Position +
        PreviousNavigationState.Velocity * PreIntegrationData.DeltaT +
        0.5 * g * PreIntegrationData.DeltaT * PreIntegrationData.DeltaT +
        PreviousNavigationState.Rwb * DeltaPositionCorrected;

    typeNavigationState Prediction(PredictedR, PredictedVelocity, PredictedPosition, PreviousNavigationState.GyroBias, PreviousNavigationState.AccelorometerBias);

    return Prediction;
}

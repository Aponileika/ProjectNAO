#include "IMU_PreIntegration.hpp"
#include "IMUPriv_PreIntegration.hpp"

typePreIntegration IntegrationState = typePreIntegration(); 

static Eigen::Vector3d g{};

void IMU_NewNavigationStateArrival(const typeNavigationState& NavigationState)
{
    IntegrationState.Reset(NavigationState);
}

bool IMU_InitializeGravity(const typeNavigationState& NavigationState,
        const typeIMUMeasurement& Measurement,
        const Eigen::Vector3d& WorldAcceleration)
{
    constexpr fp64 GravityMagnitude = 9.81;

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

    g = GravityMagnitude * GravityEstimate.normalized();

    LG_Log(LogSeverity::DATA,
            "[IMUGravityInitialization] g_W = (%.6f, %.6f, %.6f), |g| = %.6f\n",
            g.x(), g.y(), g.z(), g.norm());

    return true;
}

void IMU_IngegrationStep(const typeIMUMeasurement& Current)
{
    static typeIMUMeasurement Previous
    {
        .TimeStamp = 0.0,
        .AngularVelocity = {},
        .Acceleration = {}
    };

    static bool IsFirst =  true;
    if(IsFirst)
    {
        Previous = Current;
        IsFirst = false;
        return;
    }

    const fp64 dT = Current.TimeStamp - Previous.TimeStamp;
    // Remove bias
    const Eigen::Vector3d Omega = Current.AngularVelocity - IntegrationState.GyroBias;
    const Eigen::Vector3d Acc = Current.Acceleration - IntegrationState.AccelBias;

    const Eigen::Vector3d Phi = Omega*dT;
    const Eigen::Matrix3d dR = Sophus::SO3d::exp(Phi).matrix();

    const Eigen::Matrix3d Jr = Sophus::SO3d::leftJacobian(-Phi);

    IntegrationState.UpdateJacobians(dR, Jr, Acc, dT);
    IntegrationState.UpdateCovariance(Omega, Acc, dT);
    IntegrationState.PreIntegrate(Acc, dR, dT);

    Previous = Current;
}

void IMU_GetPreIntegratedRt(Eigen::Matrix3d& Rwb, Eigen::Vector3d& twb)
{
    static const Eigen::Vector3d Gravity(0.0, 0.0, -9.81);

    const typeNavigationState& InitialState =
        IntegrationState.InitialNavigationState;

    Rwb = InitialState.Rwb * IntegrationState.DeltaR;
    twb = InitialState.Position +
        InitialState.Velocity * IntegrationState.DeltaT +
        0.5 * Gravity * IntegrationState.DeltaT * IntegrationState.DeltaT +
        InitialState.Rwb * IntegrationState.DeltaPosition;
}

typePreIntegrationData IMU_GetLatestPreIntegrationData(void)
{
    return IntegrationState;
}

typeNavigationState IMU_PredictNavigationState(const typeNavigationState& PreviousNavigationState,
        const typePreIntegrationData& PreIntegrationData)
{
    const Eigen::Matrix3d& PredictedR = PreviousNavigationState.Rwb * PreIntegrationData.DeltaR;
    const Eigen::Vector3d& PredictedVelocity = PreviousNavigationState.Velocity + 
        g * PreIntegrationData.DeltaT + PreviousNavigationState.Rwb * PreIntegrationData.DeltaVelocity;
    const Eigen::Vector3d& PredictedPosition = PreviousNavigationState.Position + PreviousNavigationState.Velocity * PreIntegrationData.DeltaT
        + 0.5 * g * PreIntegrationData.DeltaT * PreIntegrationData.DeltaT + PreviousNavigationState.Rwb * PreIntegrationData.DeltaPosition;

    typeNavigationState Prediction(PredictedR, PredictedVelocity, PredictedPosition, PreviousNavigationState.GyroBias, PreviousNavigationState.AccelorometerBias);

    return Prediction;
}

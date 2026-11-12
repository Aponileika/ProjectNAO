#include "IMU_IMUReader.hpp"
#include "IMUPriv_IMUReader.hpp"

static typeIMUIntrinsics IMUPriv_GetConfigIntrinsics(const Dataset DatasetID)
{
    switch(DatasetID)
    {
#define X(name, rate_hz_, t_bs_, gyro_noise_density_, gyro_random_walk_, accel_noise_density_, accel_random_walk_) \
        case Dataset::name: \
        { \
            Eigen::Matrix4d T_BS; \
            T_BS << t_bs_; \
            return \
            { \
                .RateHz = rate_hz_, \
                .T_BS = T_BS, \
                .GyroscopeNoiseDensity = gyro_noise_density_, \
                .GyroscopeRandomWalk = gyro_random_walk_, \
                .AccelerometerNoiseDensity = accel_noise_density_, \
                .AccelerometerRandomWalk = accel_random_walk_ \
            }; \
        }

        DATASET_IMU_INTRINSICS

#undef X
    }

    return {};
}

static const typeIMUIntrinsics IMUPriv_Intrinsics = IMUPriv_GetConfigIntrinsics(panto_dataset);

const typeIMUIntrinsics* IMU_GetIntrinsics(void)
{
    return &IMUPriv_Intrinsics;
}

typeIMUMeasurement IMU_GetMeasurement(void)
{
    static typeIMUReader IMUReader{};
    static bool IsInit = false;
    char Buffer[1024];
    if(!IsInit)
    {
        IMUReader.Path =
            std::string(PANTO_DATASET_BASE_PATH) +
            std::string(panto_dataset_path) +
            std::string(panto_imu_path) +
            "/data.csv";
        IMUReader.FilePointer = fopen(IMUReader.Path.c_str(), "r");
        assert(IMUReader.FilePointer != nullptr);
        IsInit = true;
        fgets(Buffer, sizeof(Buffer),IMUReader.FilePointer);
    }

    if(fgets(Buffer, sizeof(Buffer),IMUReader.FilePointer) == nullptr)
    {
        return{};
    }

    std::string Line(Buffer);

    std::stringstream Stream(Line);

    std::string Token;

    typeIMUMeasurement Measurement{};

    std::getline(Stream, Token, ',');
    Measurement.TimeStamp = static_cast<fp64>(std::stod(Token)) * 1e-9;
    std::getline(Stream, Token, ',');
    Measurement.AngularVelocity.x() = std::stod(Token);
    std::getline(Stream, Token, ',');
    Measurement.AngularVelocity.y() = std::stod(Token);
    std::getline(Stream, Token, ',');
    Measurement.AngularVelocity.z() = std::stod(Token);
    std::getline(Stream, Token, ',');
    Measurement.Acceleration.x() = std::stod(Token);
    std::getline(Stream, Token, ',');
    Measurement.Acceleration.y() = std::stod(Token);
    std::getline(Stream, Token, ',');
    Measurement.Acceleration.z() = std::stod(Token);

    return Measurement;
}

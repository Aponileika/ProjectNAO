#include "IMU_IMUReader.hpp"
#include "IMUPriv_IMUReader.hpp"

typeIMUMeasurement IMU_GetMeasurement(void)
{
    static typeIMUReader IMUReader{};
    static bool IsInit = false;
    char Buffer[1024];
    if(!IsInit)
    {
        IMUReader.Path = std::string(panto_imu_path) + "data.csv";
        IMUReader.FilePointer = fopen(IMUReader.Path.c_str(), "r");
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
    Measurement.TimeStamp = static_cast<u64>(std::stoull(Token));
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



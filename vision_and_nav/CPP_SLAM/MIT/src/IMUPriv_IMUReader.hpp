#ifndef IMUPRIV_IMUREADER_HPP_
#define IMUPRIV_IMUREADER_HPP_
#include "IMU_IMUReader.hpp"

typedef struct
{
    std::string Path;
    FILE* FilePointer;
}typeIMUReader;

#endif //  IMUPRIV_IMUREADER_HPP_

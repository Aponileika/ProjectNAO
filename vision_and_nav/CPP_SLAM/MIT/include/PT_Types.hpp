#ifndef __PT_TYPES_HPP_
#define __PT_TYPES_HPP_
#include <Eigen/Dense>
#include <Config.hpp>
#include <CArenaAlloc.h>

typedef struct
{
    /**
     * Contains the undistorted point
     * */
    Eigen::Vector2d Point;
    typeDescriptor Descriptor;
    u64 MapPointID;
    u64 ID;
    u64 CellID;
}typePantoImagePoint;

typedef struct
{
    u64 KeyFrameID;
    u64 ImagePointID;
}typeObservation;

typedef struct
{
    Eigen::Vector4d Point;
    typeDescriptor Descriptor;
    std::vector<typeObservation> Observations;
    u64 ID;
}typePantoMapPoint;

#endif // __PT_TYPES_HPP_

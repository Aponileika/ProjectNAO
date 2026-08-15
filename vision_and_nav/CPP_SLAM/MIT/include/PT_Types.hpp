#ifndef __PT_TYPES_HPP_
#define __PT_TYPES_HPP_
#include <Eigen/Dense>
#include "Config.hpp"
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
    Eigen::Vector4d Point;
    typeDescriptor Descriptor;
    std::vector<u64> KeyFrameIDs;
    std::vector<u64> ImagePointIDs;
    u64 ID;
}typePantoMapPoint;

typedef struct
{
    std::vector<typePantoImagePoint> ImagePoints;
    std::array<std::vector<u64>, PANTO_CELL_SIZE*PANTO_CELL_SIZE> CellIndexingArray;
}typePantoKeypointFrame;

#endif // __PT_TYPES_HPP_

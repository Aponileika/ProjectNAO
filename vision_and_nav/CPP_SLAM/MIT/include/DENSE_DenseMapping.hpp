#ifndef DENSE_DENSEMAPPING_HPP
#define DENSE_DENSEMAPPING_HPP
#include "Config.hpp"
#include "FR_Frames.hpp"
#include "CM_Camera.hpp"
#include <Eigen/Dense>
#include <condition_variable>
#include <mutex>
#include <unordered_map>
#include <vector>
#include "PANTOVEC_PantoVector.hpp"
#include "PANTO_Utils.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/opencv.hpp"
#include "MAP_Mapping.hpp"

struct typeVoxelKey
{
    i32 X;
    i32 Y;
    i32 Z;

    bool operator==(const typeVoxelKey& VoxelKey) const = default;
};

typedef struct typeVoxelKey typeVoxelKey;

static inline typeVoxelKey DENSE_GetVoxelKey(fp32 X, fp32 Y, fp32 Z)
{
    return
    {
        static_cast<i32>(std::floor(X / DENSE_VOXEL_SIZE)),
        static_cast<i32>(std::floor(Y / DENSE_VOXEL_SIZE)),
        static_cast<i32>(std::floor(Z / DENSE_VOXEL_SIZE))
    };
}

class typeVoxelHash
{
    public:
        std::size_t operator()(const typeVoxelKey& VoxelKey) const noexcept
        {
            std::size_t Hash = std::hash<i32>{}(VoxelKey.X);

            Hash ^= std::hash<i32>{}(VoxelKey.Y) +
                0x9e3779b9U + (Hash << 6U) + (Hash >> 2U);

            Hash ^= std::hash<i32>{}(VoxelKey.Z) +
                0x9e3779b9U + (Hash << 6U) + (Hash >> 2U);

            return Hash;
        }
};

typedef struct
{
    u64 KeyFrameID;

    // Voxel grid in ccs
    Eigen::Matrix<fp32, 3, Eigen::Dynamic> MapPoints;

    // Temporary for debug
    cv::Mat DisparityColored;
}typeDenseKeyFrameMap;

typedef struct
{
    typeVoxelKey OriginKey;
    std::vector<u64> RollingOccupancy;
    bool IsInitialized;
}typeDenseVoxelOccupancyMap;

typedef struct 
{
    u64 KeyFrameID;
    cv::Mat LeftImage;
    cv::Mat RightImage;
}typeDenseData;

typedef struct
{
    typeDenseData DenseData;
    std::vector<typeKeyFrame> LocalKeyFrames;
}typeDenseLocaMapData;

typedef struct
{
    // Later the removal in localmapping must be synced with this.
    std::unordered_map<u64, typeDenseKeyFrameMap> KeyFrameMaps;
    typeSPSCQueue<cv::Mat>* VizQueue;
    typeSPSCQueue<typeDenseLocaMapData>* DenseQueue;
}typeDenseMapData;

void DENSE_DenseMapping(typeDenseMapData& MapData);
std::vector<Eigen::Vector4d> DENSE_GetDenseMap(const typePantoVector<typeKeyFrame>& GlobalKeyFrames);
std::vector<Eigen::Vector3f> DENSE_GetDenseMapPoints(void);
typeDenseVoxelOccupancyMap DENSE_GetRollingVoxelOccupancyMap(void);
Eigen::Matrix<fp32, 3, Eigen::Dynamic> DENSE_GetDensePoints(void);

#endif // DENSE_DENSEMAPPING_HPP

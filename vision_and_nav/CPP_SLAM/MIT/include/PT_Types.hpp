#ifndef __PT_TYPES_HPP_
#define __PT_TYPES_HPP_
#include <Eigen/Dense>
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"
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
#if defined(CONFIG_STEREO)
    Eigen::Vector2d RightCameraMatch;
    bool IsMatched = false;
#endif
    // TODO add this
    // fp64 DetectionScale;
}typePantoImagePoint;

struct typeDescriptorObservation
{
    u64 KeyFrameID = PANTO_ID_NOT_SET;
    u64 ImagePointID = PANTO_ID_NOT_SET;
    typeDescriptor Descriptor{};
    bool Active = false;
    bool Changed = false;
};

struct typeDescriptorDistanceCache
{
    // Slots follow the landmark's sparse observation vectors. Row i stores
    // distances to slots [0, i); inactive/recycled slots are never reused
    // without synchronizing their observation identity and descriptor.
    std::vector<typeDescriptorObservation> Observations;
    std::vector<std::vector<u16>> Distances;
    std::vector<std::size_t> ActiveIndices;
    std::vector<std::size_t> ChangedIndices;
    std::vector<u16> MedianDistances;

    typeDescriptorDistanceCache() = default;
    // Tracking/BA copies need the selected descriptor, not its quadratic
    // history. A copied landmark lazily rebuilds its own cache if modified.
    typeDescriptorDistanceCache(const typeDescriptorDistanceCache&) {}
    typeDescriptorDistanceCache& operator=(const typeDescriptorDistanceCache& Other)
    {
        if(this != &Other) Clear();
        return *this;
    }
    typeDescriptorDistanceCache(typeDescriptorDistanceCache&&) noexcept = default;
    typeDescriptorDistanceCache& operator=(typeDescriptorDistanceCache&&) noexcept = default;

    void Clear()
    {
        *this = typeDescriptorDistanceCache{};
    }

    u16 Distance(std::size_t A, std::size_t B) const
    {
        assert(A != B && Observations[A].Active && Observations[B].Active);
        return A > B ? Distances[A][B] : Distances[B][A];
    }
};

typedef struct typePantoMapPoint
{
    Eigen::Vector4d Point;
    typeDescriptor Descriptor;
    typePantoVector<u64> KeyFrameIDs;
    typePantoVector<u64> ImagePointIDs;
    u64 ID;
    // Includes stereo right-image observations that have no KeyFrameID entry.
    u64 NumObs;
    u64 NumVisible;
    u64 NumFound;
    u64 CreationAge;
    typeDescriptorDistanceCache DescriptorDistances{};
}typePantoMapPoint;

typedef struct
{
    typePantoVector<typePantoImagePoint> ImagePoints;
    std::array<typePantoVector<u64>, PANTO_NUM_IMAGE_CELLS> CellIndexingArray;
}typePantoKeypointFrame;


#endif // __PT_TYPES_HPP_

#ifndef __PT_PANTOMAPPOINTS_HPP_
#define  __PT_PANTOMAPPOINTS_HPP_
#include "CArenaAlloc.h"
#include "Config.hpp"
#include "CM_Camera.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include <Eigen/Dense>
#include <PT_Types.hpp>
#include "PANTOVEC_PantoVector.hpp"
#include "PANTO_Utils.hpp"

typePantoMapPoint PT_CreatePantoMapPoint(const Eigen::Vector4d& Point, const typeDescriptor& Descriptor, const std::pair<u64, u64>& KeyFrameIDs,
        const std::pair<u64, u64>& ImagePointIDs, const u64 ID, const u64 Age);
bool PT_IsInfront(const Eigen::Vector4d& Point, const typeCamera& Camera);

inline fp64 PT_GetFoundRatio(const typePantoMapPoint& MapPoint)
{
    return static_cast<fp64>(MapPoint.NumFound) / static_cast<fp64>(MapPoint.NumVisible);
}

inline u64 PT_GetNumObservations(const typePantoMapPoint& MapPoint)
{
    return static_cast<u64>(MapPoint.KeyFrameIDs.active_size());
}

inline typeDescriptor PT_CalculateNewDescriptor(const std::vector<typeDescriptor>& Descriptors, std::vector<i32>& Distances)
{
    assert(!Descriptors.empty());

    const std::size_t Count = Descriptors.size();
    if(Count == 1)
    {
        return Descriptors.front();
    }

    std::size_t BestDescriptorID = 0;
    i32 BestMedianDistance = std::numeric_limits<i32>::max();

    for(std::size_t i = 0; i < Count; ++i)
    {
        Distances.clear();

        for(std::size_t j = 0; j < Count; ++j)
        {
            if(i != j)
            {
                Distances.push_back(PANTO_HammingDistance(Descriptors[i], Descriptors[j]));
            }
        }

        auto Middle = Distances.begin() + Distances.size() / 2;
        std::nth_element(Distances.begin(), Middle, Distances.end());

        const i32 MedianDistance = *Middle;
        if(MedianDistance < BestMedianDistance)
        {
            BestMedianDistance = MedianDistance;
            BestDescriptorID = i;
        }

        // Hamming distances cannot be negative; later ties cannot win.
        if(BestMedianDistance == 0)
        {
            break;
        }
    }

    return Descriptors[BestDescriptorID];
}

#endif // __PT_PANTOMAPPOINTS_HPP_

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

inline typeDescriptor PT_CalculateNewDescriptor(const std::vector<typeDescriptor>& Descriptors)
{
    u64 BestDescriptorID = 0;
    fp64 BestMedianDistance = std::numeric_limits<fp64>::max();

    for(std::size_t i{}; i < Descriptors.size(); i++)
    {
        std::vector<i32> Distances;
        Distances.reserve(Descriptors.size() - 1);

        for(std::size_t j{}; j < Descriptors.size(); j++)
        {
            if(i == j)
            {
                continue;
            }

            Distances.push_back(PANTO_HammingDistance( Descriptors[i],
                        Descriptors[j]));
        }

        std::sort(Distances.begin(), Distances.end());

        const fp64 MedianDistance = Distances[Distances.size() / 2];

        if(MedianDistance < BestMedianDistance)
        {
            BestMedianDistance = MedianDistance;
            BestDescriptorID = i;
        }
    }

    return Descriptors[BestDescriptorID];
}

#endif // __PT_PANTOMAPPOINTS_HPP_

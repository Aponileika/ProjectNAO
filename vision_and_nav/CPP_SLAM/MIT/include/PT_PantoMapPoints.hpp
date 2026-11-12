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
#include <unordered_map>

typePantoMapPoint PT_CreatePantoMapPoint(const Eigen::Vector4d& Point, const typeDescriptor& Descriptor, const std::pair<u64, u64>& KeyFrameIDs,
        const std::pair<u64, u64>& ImagePointIDs, const u64 ID, const u64 Age);
bool PT_IsInfront(const Eigen::Vector4d& Point, const typeCamera& Camera);

inline fp64 PT_GetFoundRatio(const typePantoMapPoint& MapPoint)
{
    return static_cast<fp64>(MapPoint.NumFound) / static_cast<fp64>(MapPoint.NumVisible);
}

inline u64 PT_GetNumObservations(const typePantoMapPoint& MapPoint)
{
    return MapPoint.NumObs;
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

// Return the number of new Hamming evaluations. Reuse may refer to the other
// landmark's synchronized cache during fusion; both internal distance blocks
// then survive and only missing cross-observation pairs are evaluated.
template<typename typeKeyFrames>
inline u64 PT_SynchronizeMapPointDescriptorCache(typePantoMapPoint& MapPoint,
        const typeKeyFrames& KeyFrames,
        const typeDescriptorDistanceCache* Reuse = nullptr)
{
    static_assert(PANTO_DESCRIPTOR_SIZE * 8 <= 65535);
    typeDescriptorDistanceCache& Cache = MapPoint.DescriptorDistances;
    assert(Reuse != &Cache);
    assert(MapPoint.KeyFrameIDs.size() == MapPoint.ImagePointIDs.size());
    const std::size_t Count = MapPoint.KeyFrameIDs.size();
    const std::size_t PreviousCount = Cache.Distances.size();
    Cache.Observations.resize(Count);
    Cache.Distances.resize(Count);
    for(std::size_t i = PreviousCount; i < Count; ++i)
    {
        Cache.Distances[i].resize(i);
    }
    Cache.ActiveIndices.clear();
    Cache.ChangedIndices.clear();
    for(std::size_t i = 0; i < Count; ++i)
    {
        typeDescriptorObservation& Observation = Cache.Observations[i];
        Observation.Changed = false;
        assert(MapPoint.KeyFrameIDs.contains(i) == MapPoint.ImagePointIDs.contains(i));
        if(!MapPoint.KeyFrameIDs.contains(i))
        {
            Observation.Active = false;
            continue;
        }
        const u64 KeyFrameID = MapPoint.KeyFrameIDs[i];
        const u64 ImagePointID = MapPoint.ImagePointIDs[i];
        const typeDescriptor& Descriptor = KeyFrames[KeyFrameID].Points.ImagePoints[ImagePointID].Descriptor;
        if(!Observation.Active || Observation.KeyFrameID != KeyFrameID ||
                Observation.ImagePointID != ImagePointID || Observation.Descriptor != Descriptor)
        {
            Observation = {KeyFrameID, ImagePointID, Descriptor, true, true};
            Cache.ChangedIndices.push_back(i);
        }
        Cache.ActiveIndices.push_back(i);
    }

    // One observation per KF is the map's invariant. Match the image-point ID
    // and descriptor too: fusion can discard a conflicting observation from
    // the same KF, which must never donate distances to the retained one.
    std::unordered_map<u64, std::size_t> SourceIndices;
    std::vector<std::size_t> ReuseIndices;
    const std::size_t Missing = std::numeric_limits<std::size_t>::max();
    if(Reuse != nullptr)
    {
        for(const std::size_t i : Reuse->ActiveIndices)
        {
            SourceIndices.emplace(Reuse->Observations[i].KeyFrameID, i);
        }
        ReuseIndices.assign(Count, Missing);
        for(const std::size_t i : Cache.ActiveIndices)
        {
            const auto Found = SourceIndices.find(Cache.Observations[i].KeyFrameID);
            if(Found == SourceIndices.end()) continue;
            const typeDescriptorObservation& Source = Reuse->Observations[Found->second];
            if(Source.ImagePointID == Cache.Observations[i].ImagePointID &&
                    Source.Descriptor == Cache.Observations[i].Descriptor)
            {
                ReuseIndices[i] = Found->second;
            }
        }
    }

    u64 Evaluations = 0;
    for(const std::size_t i : Cache.ChangedIndices)
    {
        for(const std::size_t j : Cache.ActiveIndices)
        {
            if(i == j || (Cache.Observations[j].Changed && j < i)) continue;
            u16 Distance;
            if(Reuse != nullptr && ReuseIndices[i] != Missing && ReuseIndices[j] != Missing)
            {
                Distance = Reuse->Distance(ReuseIndices[i], ReuseIndices[j]);
            }
            else
            {
                Distance = static_cast<u16>(PANTO_HammingDistance(
                            Cache.Observations[i].Descriptor, Cache.Observations[j].Descriptor));
                ++Evaluations;
            }
            Cache.Distances[std::max(i, j)][std::min(i, j)] = Distance;
        }
    }

    return Evaluations;
}

template<typename typeKeyFrames>
inline u64 PT_UpdateMapPointDescriptor(typePantoMapPoint& MapPoint,
        const typeKeyFrames& KeyFrames,
        const typeDescriptorDistanceCache* Reuse = nullptr)
{
    const u64 Evaluations = PT_SynchronizeMapPointDescriptorCache(MapPoint, KeyFrames, Reuse);
    typeDescriptorDistanceCache& Cache = MapPoint.DescriptorDistances;
    if(Cache.ActiveIndices.empty()) return Evaluations;
    std::size_t Best = Cache.ActiveIndices.front();
    u16 BestMedian = std::numeric_limits<u16>::max();
    for(const std::size_t i : Cache.ActiveIndices)
    {
        Cache.MedianDistances.clear();
        for(const std::size_t j : Cache.ActiveIndices)
        {
            if(i != j) Cache.MedianDistances.push_back(Cache.Distance(i, j));
        }
        if(Cache.MedianDistances.empty()) break;
        auto Middle = Cache.MedianDistances.begin() + Cache.MedianDistances.size() / 2;
        std::nth_element(Cache.MedianDistances.begin(), Middle, Cache.MedianDistances.end());
        if(*Middle < BestMedian)
        {
            Best = i;
            BestMedian = *Middle;
        }
        // Preserve the existing upper median, sparse-slot order and first tie.
        if(BestMedian == 0) break;
    }
    MapPoint.Descriptor = Cache.Observations[Best].Descriptor;
    return Evaluations;
}

#endif // __PT_PANTOMAPPOINTS_HPP_

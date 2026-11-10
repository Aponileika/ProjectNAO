#include "MAP_Mapping.hpp"
#include <cstdlib>
#include <iostream>
#include <unordered_set>

void LG_Log(LogSeverity, const char*, ...) {}
void LG_LogData(const char*, ...) {}
#include "../src/PANTO_Utils.cpp"
#include "../src/MAP_Mapping.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

template<typename T> concept HasKeyFramePayload = requires(T Object) { Object.KeyFrames; };
static_assert(!HasKeyFramePayload<typeLocalMapTracking>);

struct Fixture
{
    typeGlobalMap Map{};
    typeCovisibilityGraph Graph{};

    explicit Fixture(u64 Count)
    {
        Map.KeyFrames.reserve(Count);
        Map.MapPoints.reserve(Count + 32);
        for(u64 i = 0; i < Count; ++i)
        {
            typePantoMapPoint Point{};
            Point.ID = i;
            Point.Point = Eigen::Vector4d(i, 1, 3, 1);
            Map.MapPoints.push_back(std::move(Point));
            typeKeyFrame Frame{};
            Frame.ID = i;
            typePantoImagePoint Image{};
            Image.ID = 0;
            Image.MapPointID = i;
            Frame.Points.ImagePoints.push_back(Image);
            Map.KeyFrames.push_back(std::move(Frame));
            Graph.CovisibilityGraph.push_back({});
        }
    }

    u64 AddVotingPoint(u64 First, u64 Count)
    {
        typePantoMapPoint Point{};
        Point.ID = Map.MapPoints.size();
        Point.Point = Eigen::Vector4d(0, 0, 4, 1);
        for(u64 ID = First; ID < First + Count; ++ID)
        {
            typePantoImagePoint Image{};
            Image.ID = Map.KeyFrames[ID].Points.ImagePoints.size();
            Image.MapPointID = Point.ID;
            Point.KeyFrameIDs.push_back(ID);
            Point.ImagePointIDs.push_back(Image.ID);
            Map.KeyFrames[ID].Points.ImagePoints.push_back(Image);
            ++Point.NumObs;
        }
        const u64 ID = Map.MapPoints.push_back(std::move(Point));
        PT_UpdateMapPointDescriptor(Map.MapPoints[ID], Map.KeyFrames);
        return ID;
    }
};

static bool Contains(const typeLocalMapTracking& Snapshot, u64 ID)
{
    return std::any_of(Snapshot.MapPoints.begin(), Snapshot.MapPoints.end(),
            [ID](const typePantoMapPoint& Point) { return Point.ID == ID; });
}

static typeKeyFrame Query(const std::vector<u64>& IDs)
{
    typeKeyFrame Frame{};
    for(const u64 ID : IDs)
    {
        typePantoImagePoint Image{};
        Image.MapPointID = ID;
        Frame.Points.ImagePoints.push_back(Image);
    }
    return Frame;
}

static void TestDirectLookupAndSparseIDs()
{
    Fixture F(2048);
    F.Map.MapPoints.remove(31);
    const typeKeyFrame Frame = Query({1900, 7, 31, PANTO_ID_NOT_SET, 999999});
    const auto Points = MAP_GetLastFrameMapPoints(F.Map.MapPoints, Frame);
    Require(Points.size() == 2 && Points[0].ID == 1900 && Points[1].ID == 7,
            "global lookup follows only the KF IDs and skips removed/out-of-range points");
    const auto ThroughMap = MAP_GetLastFrameMapPoints(F.Map, Frame);
    Require(ThroughMap.size() == Points.size() && ThroughMap[0].Point == Points[0].Point,
            "both global-map entry points use indexed lookup");
    const auto Compact = MAP_GetLastFrameMapPoints(Points, Query({7, 1900}));
    Require(Compact.size() == 2 && Compact[0].ID == 7 && Compact[1].ID == 1900,
            "compact local vectors still resolve non-indexed global IDs correctly");
    const u64 Invalid = 999999;
    Require(!F.Map.MapPoints.contains(Invalid), "u64 lookups use the bounds-checked contains overload");
}

static void TestBoundedHistoryAndSparseObservations()
{
    Fixture F(4096);
    const u64 ID = F.AddVotingPoint(0, 4096);
    typeTrackingMapSearchStatistics Search;
    auto Snapshot = MAP_CreateLocalMapTracking(F.Map, F.Graph, Query({ID}), &Search);
    Require(Search.ObservationProbes == PANTO_TRACKING_OBSERVER_SAMPLES_PER_POINT,
            "long observation histories have a fixed probe budget");
    Require(Search.SelectedKeyFrames == PANTO_TRACKING_OBSERVER_SAMPLES_PER_POINT && Contains(Snapshot, 4095),
            "stratified voting includes the newest observation");
    std::unordered_set<u64> IDs;
    for(const typePantoMapPoint& Point : Snapshot.MapPoints)
    {
        Require(IDs.insert(Point.ID).second, "tracking points are deduplicated across KFs");
        Require(Point.KeyFrameIDs.empty() && Point.ImagePointIDs.empty() && Point.DescriptorDistances.Observations.empty(),
                "tracking copies omit observation and descriptor histories");
    }
    auto& Point = F.Map.MapPoints[ID];
    for(u64 Slot = 0; Slot < 4096; ++Slot)
    {
        if(Slot % 1000 != 0)
        {
            Point.KeyFrameIDs.remove(Slot);
            Point.ImagePointIDs.remove(Slot);
            --Point.NumObs;
        }
    }
    PT_UpdateMapPointDescriptor(Point, F.Map.KeyFrames);
    Snapshot = MAP_CreateLocalMapTracking(F.Map, F.Graph, Query({ID}), &Search);
    Require(Search.ObservationProbes == 5 && Search.SelectedKeyFrames == 5 && Contains(Snapshot, 0),
            "active-slot sampling skips thousands of tombstones without scanning them");
    Require(Point.KeyFrameIDs.active_size() == 5, "snapshot creation leaves the live observation history intact");
}

static void TestCandidateAndOutputCaps()
{
    Fixture F(512);
    std::vector<u64> IDs;
    for(u64 i = 0; i < 12; ++i) IDs.push_back(F.AddVotingPoint(i * 32, 32));
    typeTrackingMapSearchStatistics Search;
    const auto Snapshot = MAP_CreateLocalMapTracking(F.Map, F.Graph, Query(IDs), &Search);
    Require(Search.ObservationProbes == 384 && Search.CandidateKeyFrames == PANTO_TRACKING_MAX_CANDIDATE_KFS,
            "candidate admission is bounded before sorting");
    Require(Search.SelectedKeyFrames == PANTO_MAX_LOCAL_TRACKING_MAP_SIZE,
            "output KF limit is retained");
    Require(Contains(Snapshot, 0) && Contains(Snapshot, 79) && !Contains(Snapshot, 80),
            "small-history vote ties retain ascending-ID ordering");
}

static void TestBoundedNeighborExpansion()
{
    Fixture F(1024);
    const u64 ID = F.AddVotingPoint(0, 1);
    for(u64 Neighbor = 1; Neighbor < 1024; ++Neighbor) F.Graph.CovisibilityGraph[0][Neighbor] = Neighbor;
    typeTrackingMapSearchStatistics Search;
    const auto Snapshot = MAP_CreateLocalMapTracking(F.Map, F.Graph, Query({ID}), &Search);
    Require(Search.NeighborProbes == PANTO_TRACKING_NEIGHBOR_PROBES_PER_KF && Search.SelectedKeyFrames == 2,
            "dense covisibility graphs have a fixed neighbor-search budget");
    Require(Contains(Snapshot, 0), "seed points survive neighbor expansion");
}

int main()
{
    TestDirectLookupAndSparseIDs();
    TestBoundedHistoryAndSparseObservations();
    TestCandidateAndOutputCaps();
    TestBoundedNeighborExpansion();
    std::cout << "Bounded tracking snapshots and indexed map-point lookup tests passed\n";
}

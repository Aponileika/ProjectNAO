#include "MAP_Mapping.hpp"
#include "Config.hpp"
#include "GRAPH_PantoGraph.hpp"
#include "IMU_IMUReader.hpp"
#include "MAPPriv_Mapping.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "PANTO_Utils.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "PT_PantoMapPoints.hpp"
#include "PT_Types.hpp"
#include "opencv2/flann.hpp"
#include <MIT/src/INITPriv_InitializeSLAM.hpp>
#include <limits>
#include <unordered_map>

typeMappingData MappingData = 
{
    .RecentMapPointsCulled = 0,
    .KeyFramesCulled = 0,
    .MapPointsCulled = 0,
    .MapPointFusionObservations = 0,
    .MapPointFusions = 0,

    .ObservationEdgesCulled = 0,
    .NumObservationEdgesPixelErrorHigh = 0,
    .NumObservationEdgesFailedProjection = 0,
    .SumPixelErrorRemovedPixels = 0.0,
    .SquaredSumPixelErrorRemovedPixels = 0.0
};

void MAP_InitializeFromGT(const typeNavigationState& FirstNavState, const typeNavigationState& SecondNavState,
        const typePantoFrame& FirstFrame, const typePantoFrame& SecondFrame, const std::vector<typeIMUMeasurement>& IMUMeasurementsFrame1to2, typeGlobalMap* GlobalMap)
{
    typeKeyFrame FirstKF = KEY_CreateKeyFrame(FirstNavState, FirstFrame, 0);
    typeKeyFrame SecondKF = KEY_CreateKeyFrame(SecondNavState, SecondFrame, 1);

    FirstKF.Measurements = {};
    SecondKF.Measurements = IMUMeasurementsFrame1to2;

    const std::vector<typePantoMapPoint>& MapPoints = KEY_InsertNewMapPoints(FirstKF, SecondKF, GlobalMap->Age);

    (void) MAP_AppendKeyFrame(GlobalMap, FirstKF);
    (void) MAP_AppendKeyFrame(GlobalMap, SecondKF);

    for(const typePantoMapPoint& MapPoint : MapPoints)
    {
        const std::vector<u64>& KeyFrameIDs   = MapPoint.KeyFrameIDs;
        const std::vector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;
        typeKeyFrame& KeyFrame1 = GlobalMap->KeyFrames[KeyFrameIDs[0]];
        typeKeyFrame& KeyFrame2 = GlobalMap->KeyFrames[KeyFrameIDs[1]];

        typePantoImagePoint& ImagePoint1 = KeyFrame1.Points.ImagePoints[ImagePointIDs[0]];
        typePantoImagePoint& ImagePoint2 = KeyFrame2.Points.ImagePoints[ImagePointIDs[1]];

        const u64 MapPointID = GlobalMap->MapPoints.push_back(MapPoint);

        GlobalMap->MapPoints[MapPointID].ID = MapPointID;

        ImagePoint1.MapPointID = MapPointID;
        ImagePoint2.MapPointID = MapPointID;
    }
}

u64 MAP_AppendKeyFrame(typeGlobalMap* GlobalMap, const typeKeyFrame& KeyFrame)
{
    const u64 ID = GlobalMap->KeyFrames.push_back(KeyFrame);
    GlobalMap->KeyFrames[ID].ID = ID;
    GlobalMap->Age++;
    GlobalMap->Revision++;
    return ID;
}

typeLocalMapTracking MAP_CreateLocalMapTracking(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph,
    const typeKeyFrame& CurrentFrame)
{
    LG_Log( LogSeverity::DBG,
        "[MAP_CreateLocalMapTracking] Creating tracking-local map\n");

    typeLocalMapTracking LocalMap{};

    /*
     * Keyframe IDs may be sparse, so do not allocate a vector using
     * active_size() and index it with KeyFrameID.
     */
    std::unordered_map<u64, u64> KeyFrameVotes;
    KeyFrameVotes.reserve( CurrentFrame.Points.ImagePoints.size());


    for(const typePantoImagePoint& ImagePoint : CurrentFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;

        if(MapPointID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        if(!GlobalMap.MapPoints.contains(MapPointID))
        {
            continue;
        }

        const typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];

        for(const u64 ObservingKeyFrameID : MapPoint.KeyFrameIDs)
        {
            ++KeyFrameVotes[ObservingKeyFrameID];
        }
    }


    struct typeKeyFrameCandidate
    {
        u64 KeyFrameID;
        u64 VoteCount;
    };

    std::vector<typeKeyFrameCandidate> KeyFrameCandidates;

    KeyFrameCandidates.reserve( KeyFrameVotes.size());

    for(const auto& VoteEntry : KeyFrameVotes)
    {
        const u64 KeyFrameID = VoteEntry.first;

        const u64 VoteCount = VoteEntry.second;

        /*
         * Observation lists can contain references to keyframes that have
         * since been removed. Filter those references here.
         */
        if(!GlobalMap.KeyFrames.contains(KeyFrameID))
        {
            continue;
        }

        KeyFrameCandidates.push_back({ .KeyFrameID = KeyFrameID, .VoteCount = VoteCount
        });
    }

    std::sort(
        KeyFrameCandidates.begin(),
        KeyFrameCandidates.end(),
        [](
            const typeKeyFrameCandidate& Left,
            const typeKeyFrameCandidate& Right)
        {
            if(Left.VoteCount != Right.VoteCount)
            {
                return Left.VoteCount > Right.VoteCount;
            }

            return Left.KeyFrameID < Right.KeyFrameID;
        });

    std::vector<u64> LocalKeyFrameIDs;
    LocalKeyFrameIDs.reserve( PANTO_MAX_LOCAL_TRACKING_MAP_SIZE);

    std::unordered_set<u64> AddedKeyFrames;
    AddedKeyFrames.reserve( PANTO_MAX_LOCAL_TRACKING_MAP_SIZE);

    for(const typeKeyFrameCandidate& Candidate : KeyFrameCandidates)
    {
        if(LocalKeyFrameIDs.size() >= PANTO_MAX_LOCAL_TRACKING_MAP_SIZE)
        {
            break;
        }

        if(AddedKeyFrames.insert( Candidate.KeyFrameID).second)
        {
            LocalKeyFrameIDs.push_back( Candidate.KeyFrameID);

            LG_Log(
                LogSeverity::DBG,
                "[MAP_CreateLocalMapTracking] "
                "Selected voted KeyFrame %llu "
                "with %llu votes\n",
                Candidate.KeyFrameID,
                Candidate.VoteCount);
        }
    }

    const std::size_t NumberOfSeedKeyFrames = LocalKeyFrameIDs.size();

    for(std::size_t SeedIndex = 0; SeedIndex < NumberOfSeedKeyFrames; ++SeedIndex)
    {
        if(LocalKeyFrameIDs.size() >= PANTO_MAX_LOCAL_TRACKING_MAP_SIZE)
        {
            break;
        }

        const u64 SeedKeyFrameID = LocalKeyFrameIDs[SeedIndex];

        const typeCovisibility MostCovisibleFrame =
            GRAPH_GetMostCovisibleFrame( CovisibilityGraph, SeedKeyFrameID);

        const u64 NeighbourKeyFrameID = MostCovisibleFrame.KeyFrameID;

        LG_Log(
            LogSeverity::DBG,
            "[MAP_CreateLocalMapTracking] "
            "Most covisible neighbour of %llu is %llu\n",
            SeedKeyFrameID,
            NeighbourKeyFrameID);

        if(NeighbourKeyFrameID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        if(!GlobalMap.KeyFrames.contains( NeighbourKeyFrameID))
        {
            continue;
        }

        if(AddedKeyFrames.insert( NeighbourKeyFrameID).second)
        {
            LocalKeyFrameIDs.push_back( NeighbourKeyFrameID);
        }
    }


    LocalMap.KeyFrames.reserve( LocalKeyFrameIDs.size());

    std::unordered_set<u64> AddedMapPoints;

    AddedMapPoints.reserve( CurrentFrame.Points.ImagePoints.size() * 2);

    for(const u64 KeyFrameID : LocalKeyFrameIDs)
    {
        if(!GlobalMap.KeyFrames.contains(KeyFrameID))
        {
            continue;
        }

        const typeKeyFrame& GlobalKeyFrame = GlobalMap.KeyFrames[KeyFrameID];

        LocalMap.KeyFrames.push_back( GlobalKeyFrame);

        LG_Log(
            LogSeverity::DBG,
            "[MAP_CreateLocalMapTracking] "
            "Gathering map points from KeyFrame %llu\n",
            KeyFrameID);

        for(const typePantoImagePoint& ImagePoint : GlobalKeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }

            if(!AddedMapPoints.insert( MapPointID).second)
            {
                continue;
            }

            if(!GlobalMap.MapPoints.contains(MapPointID))
            {
                AddedMapPoints.erase( MapPointID);

                continue;
            }

            const typePantoMapPoint& GlobalMapPoint = GlobalMap.MapPoints[MapPointID];

            LocalMap.MapPoints.push_back( GlobalMapPoint);
        }
    }


    LG_Log(
        LogSeverity::DBG,
        "[MAP_CreateLocalMapTracking] "
        "Created local snapshot with %zu KeyFrames "
        "and %zu MapPoints\n",
        LocalMap.KeyFrames.size(),
        LocalMap.MapPoints.size());

    return LocalMap;
}

typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const u64 LatestKeyFrameID)
{
    typeLocalMap LocalMap{};
    std::vector<u64> LocalKeyFrameIDs;
    std::vector<u64> FixedKeyFrameIDs;
    std::vector<u64> LocalMapPointIDs;
#if defined(CONFIG_IMU)
    LocalKeyFrameIDs.reserve(PANTO_NUM_TEMPORALLY_CONNECTED_KFS_LOCAL_BA);
    u64 IMUAnchorID = PANTO_ID_NOT_SET;

    u64 CurrentKeyFrameID = LatestKeyFrameID;

    for(u64 i = 0; i < PANTO_NUM_TEMPORALLY_CONNECTED_KFS_LOCAL_BA; i++)
    {
        LocalKeyFrameIDs.push_back(CurrentKeyFrameID);

        const u64 PreviousKeyFrameID =
            GlobalMap.KeyFrames[CurrentKeyFrameID].PreviousKFID;

        if(PreviousKeyFrameID == PANTO_ID_NOT_SET)
        {
            // CurrentKeyFrameID is the root KF. Must be fixed
            LocalKeyFrameIDs.pop_back();
            IMUAnchorID = CurrentKeyFrameID;
            break;
        }

        CurrentKeyFrameID = PreviousKeyFrameID;
    }

    if(IMUAnchorID == PANTO_ID_NOT_SET)
    {
        IMUAnchorID =
            GlobalMap.KeyFrames[LocalKeyFrameIDs.back()].PreviousKFID;
    }

    assert(IMUAnchorID != PANTO_ID_NOT_SET);
    assert(GlobalMap.KeyFrames.contains(IMUAnchorID));
    LocalMap.IMUAnchor = GlobalMap.KeyFrames[IMUAnchorID].NavigationState;

    // The temporal anchor is fixed by the inertial problem, but its
    // observations must still constrain the local map points visually.
    FixedKeyFrameIDs.push_back(IMUAnchorID);

    std::unordered_set<u64> MapPointInLocalMap;
    for(const u64 KeyFrameID : LocalKeyFrameIDs)
    {
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }
            if(MapPointInLocalMap.insert(ImagePoint.MapPointID).second)
            {
                LocalMapPointIDs.push_back(ImagePoint.MapPointID);
            }
        }
    }

    const std::vector<typeCovisibility> ExternalKeyFrames =
        GRAPH_GetTopNExternalCovisibleFrames(
                CovisibilityGraph,
                LocalKeyFrameIDs,
                PANTO_MAX_FIXED_KFS_LOCAL_BA - 1,
                IMUAnchorID);

    for(const typeCovisibility& ExternalKeyFrame : ExternalKeyFrames)
    {
        FixedKeyFrameIDs.push_back(ExternalKeyFrame.KeyFrameID);
    }

#else
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);
    std::unordered_set<u64> KeyFrameInLocalMap;

    LocalKeyFrameIDs.push_back(LatestKeyFrameID);
    KeyFrameInLocalMap.insert(LatestKeyFrameID);

    for(const typeCovisibility& Covisibility : MostCovisible)
    {
        LocalKeyFrameIDs.push_back(Covisibility.KeyFrameID);
        KeyFrameInLocalMap.insert(Covisibility.KeyFrameID);
    }

    std::unordered_set<u64> MapPointInLocalMap;

    for(const u64 KeyFrameID : LocalKeyFrameIDs)
    {
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }
            if(MapPointInLocalMap.insert(ImagePoint.MapPointID).second)
            {
                LocalMapPointIDs.push_back(ImagePoint.MapPointID);
            }
        }
    }

    for(const u64 LocalMapPointID : LocalMapPointIDs)
    {
        typePantoVector<u64> KeyFrameIDs = GlobalMap.MapPoints[LocalMapPointID].KeyFrameIDs;

        for(const u64 KeyFrameID : KeyFrameIDs)
        {
            if(KeyFrameInLocalMap.insert(KeyFrameID).second)
            {
                FixedKeyFrameIDs.push_back(KeyFrameID);
            }
        }
    }

#endif // CONFIG_IMU

    LocalMap.KeyFrames.reserve(LocalKeyFrameIDs.size());
    for(const u64 KeyFrameID : LocalKeyFrameIDs)
    {
        assert(GlobalMap.KeyFrames.contains(KeyFrameID));
        LocalMap.KeyFrames.push_back(GlobalMap.KeyFrames[KeyFrameID]);
    }

    LocalMap.FixedKeyFrames.reserve(FixedKeyFrameIDs.size());
    for(const u64 KeyFrameID : FixedKeyFrameIDs)
    {
        assert(GlobalMap.KeyFrames.contains(KeyFrameID));
        LocalMap.FixedKeyFrames.push_back(GlobalMap.KeyFrames[KeyFrameID]);
    }

    LocalMap.MapPoints.reserve(LocalMapPointIDs.size());
    for(const u64 MapPointID : LocalMapPointIDs)
    {
        assert(GlobalMap.MapPoints.contains(MapPointID));
        LocalMap.MapPoints.push_back(GlobalMap.MapPoints[MapPointID]);
    }

    return LocalMap;
}

bool MAP_CommitLocalMap(typeGlobalMap* GlobalMap,
        const typeLocalMap& LocalMap)
{
    assert(GlobalMap != nullptr);
    std::lock_guard<std::mutex> Lock(GlobalMap->Mutex);

    for(const typeKeyFrame& LocalKeyFrame : LocalMap.KeyFrames)
    {
        if(!GlobalMap->KeyFrames.contains(LocalKeyFrame.ID))
        {
            continue;
        }

        typeKeyFrame& GlobalKeyFrame = GlobalMap->KeyFrames[LocalKeyFrame.ID];
        GlobalKeyFrame.Camera.Pose = LocalKeyFrame.Camera.Pose;
#if defined(CONFIG_IMU)
        GlobalKeyFrame.NavigationState = LocalKeyFrame.NavigationState;
#endif
    }

    for(const typePantoMapPoint& LocalMapPoint : LocalMap.MapPoints)
    {
        if(GlobalMap->MapPoints.contains(LocalMapPoint.ID))
        {
            GlobalMap->MapPoints[LocalMapPoint.ID].Point = LocalMapPoint.Point;
        }
    }

    GlobalMap->Revision++;

    return true;
}

void MAP_CommitTrackingStatistics(typeGlobalMap* GlobalMap,
        const typeLocalMapInfo& LocalMapInfo)
{
    assert(GlobalMap != nullptr);
    std::lock_guard<std::mutex> Lock(GlobalMap->Mutex);

    for(const u64 MapPointID : LocalMapInfo.VisibleMapPointIDs)
    {
        if(GlobalMap->MapPoints.contains(MapPointID))
        {
            GlobalMap->MapPoints[MapPointID].NumVisible++;
        }
    }
    for(const u64 MapPointID : LocalMapInfo.FoundMapPointIDs)
    {
        if(GlobalMap->MapPoints.contains(MapPointID))
        {
            GlobalMap->MapPoints[MapPointID].NumFound++;
        }
    }
}

typePantoVector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame)
{
    const typePantoKeypointFrame& LastKeyFramePoints = LastKeyFrame.Points;
    typePantoVector<typePantoMapPoint> LastKeyFrameMapPoints;
    const typePantoVector<typePantoMapPoint>& MapPoints = Map.MapPoints;

    for(const typePantoImagePoint& ImagePoint : LastKeyFramePoints.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET &&
           MapPoints.contains(ImagePoint.MapPointID))
        {
            LastKeyFrameMapPoints.push_back(MapPoints[ImagePoint.MapPointID]);
        }
    }
    return LastKeyFrameMapPoints;
}

std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(
        const std::vector<typePantoMapPoint>& MapPoints,
        const typeKeyFrame& LastKeyFrame)
{
    std::unordered_map<u64, const typePantoMapPoint*> MapPointsByID;
    MapPointsByID.reserve(MapPoints.size());
    for(const typePantoMapPoint& MapPoint : MapPoints)
    {
        MapPointsByID.emplace(MapPoint.ID, &MapPoint);
    }

    std::vector<typePantoMapPoint> LastFrameMapPoints;
    LastFrameMapPoints.reserve(LastKeyFrame.Points.ImagePoints.active_size());
    for(const typePantoImagePoint& ImagePoint :
            LastKeyFrame.Points.ImagePoints)
    {
        const auto MapPointIt = MapPointsByID.find(ImagePoint.MapPointID);
        if(MapPointIt != MapPointsByID.end())
        {
            LastFrameMapPoints.push_back(*MapPointIt->second);
        }
    }
    return LastFrameMapPoints;
}

typeLocalMapInfo MAP_MatchMapPointLocalMap(typeLocalMapTracking& LocalMap,
        typeKeyFrame& NewKeyFrame)
{
    LG_Log(LogSeverity::DBG, "[MAP_MatchMapPointLocalMap] Getting median scene depth");
    const fp64 MedianDepth =
        KEY_GetLocalMapMedianDepth(NewKeyFrame, LocalMap.MapPoints);

    const typeCamera& Camera = NewKeyFrame.Camera;
    LG_Log(LogSeverity::DBG, "[MAP_MatchMapPointLocalMap] Matching mappoints to keyframe");
    u64 NumProjectedMapPoints = 0;
    std::vector<u64> VisibleMapPointIDs;
    std::vector<u64> FoundMapPointIDs;
    const u64 NumTrackedMapPoints = MAP_MatchMapPointsToKeyFrame(
            NewKeyFrame.Points, LocalMap.MapPoints,
            Camera, &NumProjectedMapPoints,
            &VisibleMapPointIDs, &FoundMapPointIDs);

    const fp64 TrackingRatio = NumProjectedMapPoints > 0
        ? static_cast<fp64>(NumTrackedMapPoints) / static_cast<fp64>(NumProjectedMapPoints)
        : 0.0;

    LG_Log(LogSeverity::DBG,
            "[MAP_MatchMapPointLocalMap] Tracking ratio = %llu/%llu = %.6f\n",
            static_cast<unsigned long long>(NumTrackedMapPoints),
            static_cast<unsigned long long>(NumProjectedMapPoints),
            TrackingRatio);

    typeLocalMapInfo LocalMapInfo = 
    {
        .TrackedRatio = TrackingRatio,
        .MedianDepth = MedianDepth,
        .VisibleMapPointIDs = std::move(VisibleMapPointIDs),
        .FoundMapPointIDs = std::move(FoundMapPointIDs)
    };

    return LocalMapInfo;
}

/*several problems with this function:
 * 1. no temporal check in inertial
 * 2. num observations for a mappoint is a weak requirement, no descriptor checks for its observations
 * 3. culls even when the globalmap is tiny
 *    what chat gpt thinks:
     *    CULLING CANDIDATE SET
     *    current KF
     *    ↓
     *    all local covisible KFs
     *    ↓
     *    test redundancy

     *    VI safeguards:
     *    - map > 21 KFs
     *    - don't remove newest ~2 KFs
     *    - keep recent 21-KF temporal window relatively dense

     *    Resulting Prev → Next IMU interval:

     *    recent region:
     *    t < 0.5 s

     *    older than recent 21-KF window:
     *    t < 3.0 s, once IMU initialized

     *    special early inertial low-motion case:
     *    t < 3.0 s
     *    and displacement < 0.02 m
    */
void MAP_CullLocalMap(typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph, const u64 CurrentFrameID)
{
    assert(GlobalMap->KeyFrames.contains(CurrentFrameID));
    assert(CovisibilityGraph->CovisibilityGraph.contains(CurrentFrameID));

    std::vector<u64> CovisibleKeyFrameIDs;
    CovisibleKeyFrameIDs.reserve(CovisibilityGraph->CovisibilityGraph[CurrentFrameID].size());

    for(const auto& [KeyFrameID, Weight] : CovisibilityGraph->CovisibilityGraph[CurrentFrameID])
    {
        if(Weight == 0)
        {
            continue;
        }

        assert(GlobalMap->KeyFrames.contains(KeyFrameID));
        CovisibleKeyFrameIDs.push_back(KeyFrameID);
    }

    LG_Log(LogSeverity::DBG,
            "[MAP_CullLocalMap] Evaluating %zu covisible keyframes connected to KF %llu\n",
            CovisibleKeyFrameIDs.size(),
            CurrentFrameID);

    std::vector<u64> CulledKeyFrameIDs;
    for(const u64 KeyFrameID : CovisibleKeyFrameIDs)
    {
        if(KeyFrameID == CurrentFrameID)
        {
            continue;
        }

        if(KeyFrameID == 0)
        {
            continue;
        }

        typeKeyFrame& KeyFrame = GlobalMap->KeyFrames[KeyFrameID];

#if defined(CONFIG_IMU)
        if(KeyFrame.PreviousKFID == PANTO_ID_NOT_SET ||
           KeyFrame.NextKFID == PANTO_ID_NOT_SET ||
           !GlobalMap->KeyFrames.contains(KeyFrame.PreviousKFID) ||
           !GlobalMap->KeyFrames.contains(KeyFrame.NextKFID))
        {
            continue;
        }
#endif

        u64 NumMapPoints = 0;
        u64 NumRedundantMapPoints = 0;

        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }

            NumMapPoints++;

            const typePantoMapPoint& MapPoint = GlobalMap->MapPoints[MapPointID];
            const u64 OtherObservations = MapPoint.KeyFrameIDs.active_size() - 1;

            if(OtherObservations >= 3)
            {
                NumRedundantMapPoints++;
            }
        }

        if(NumMapPoints == 0)
        {
            continue;
        }

        if(static_cast<fp64>(NumRedundantMapPoints) > 0.9 * static_cast<fp64>(NumMapPoints))
        {
            CulledKeyFrameIDs.push_back(KeyFrameID);
            MappingData.KeyFramesCulled++;
        }
    }

    for(const u64 CulledID : CulledKeyFrameIDs)
    {
        typeKeyFrame& KeyFrame = GlobalMap->KeyFrames[CulledID];

#if defined(CONFIG_IMU)
        typeKeyFrame& PreviousKeyFrame = GlobalMap->KeyFrames[KeyFrame.PreviousKFID];
        typeKeyFrame& NextKeyFrame = GlobalMap->KeyFrames[KeyFrame.NextKFID];
        KEY_ReIntegrate(NextKeyFrame, KeyFrame.Measurements);
        PreviousKeyFrame.NextKFID = NextKeyFrame.ID;
        NextKeyFrame.PreviousKFID = PreviousKeyFrame.ID;
#endif
        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }

            if(!GlobalMap->MapPoints.contains(MapPointID))
            {
                continue;
            }

            typePantoMapPoint& MapPoint = GlobalMap->MapPoints[MapPointID];
            for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
            {
                if(!MapPoint.KeyFrameIDs.contains(i) || !MapPoint.ImagePointIDs.contains(i))
                {
                    continue;
                }

                if(MapPoint.KeyFrameIDs[i] == CulledID)
                {
                    MapPoint.KeyFrameIDs.remove(i);
                    MapPoint.ImagePointIDs.remove(i);
                    break;
                }
            }
        }

        GRAPH_CullKeyFrame(CovisibilityGraph, CulledID);
        GlobalMap->KeyFrames.remove(CulledID);
        LG_Log(LogSeverity::DBG, "[MAP_CullLocalMap] culled keyframe %llu\n", CulledID);
    }

    LG_Log(LogSeverity::DBG, "[MAP_CullLocalMap] Culled %zu kfs",
            CulledKeyFrameIDs.size());
}

void MAP_CullRecentMapPoints(std::unordered_set<u64>& RecentMapPointIndexes,
        typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph)
{
    LG_Log( LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culling recent mappoints\n");
    std::vector<u64> RemoveRecentIndexes;
    RemoveRecentIndexes.reserve(RecentMapPointIndexes.size());
    std::vector<u64> MapPointRemovalIndexes;
    MapPointRemovalIndexes.reserve(RecentMapPointIndexes.size());

    u64 NumRemoved = 0;

    for(const u64 MapPointID : RecentMapPointIndexes)
    {
        if(!GlobalMap->MapPoints.contains(MapPointID))
        {
            RemoveRecentIndexes.push_back(MapPointID);
            continue;
        }

        typePantoMapPoint& MapPoint = GlobalMap->MapPoints[MapPointID];
        const u64 Age = GlobalMap->Age - MapPoint.CreationAge;
        const u64 NumObservations = static_cast<u64>(MapPoint.KeyFrameIDs.active_size());
        if(PT_GetFoundRatio(MapPoint) < PANTO_MIN_FOUND_RATIO)
        {
            RemoveRecentIndexes.push_back(MapPointID);
            MapPointRemovalIndexes.push_back(MapPointID);
            NumRemoved++;
            MappingData.RecentMapPointsCulled++;
            continue;
        }
        else if(Age >= 2 && NumObservations <= 2)
        {
            RemoveRecentIndexes.push_back(MapPointID);
            MapPointRemovalIndexes.push_back(MapPointID);
            NumRemoved++;
            MappingData.RecentMapPointsCulled++;
            continue;
        }
        else if(Age >= 3)
        {
            RemoveRecentIndexes.push_back(MapPointID);
        }
    }

    for(const u64 Index : RemoveRecentIndexes)
    {
        RecentMapPointIndexes.erase(Index);
    }

    for(const u64 Index : MapPointRemovalIndexes)
    {
            MAPPriv_CullRecentMapPoint(Index, GlobalMap, CovisibilityGraph); 
    }

    LG_Log(LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culled %llu mappoints\n", NumRemoved);
}

void MAP_CullObservationEdges(typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph)
{
    const std::size_t InitialMapPointCount = GlobalMap->MapPoints.active_size();

    u64 NumMapPointsWithBadEdges = 0;
    u64 NumCulledObservationEdges = 0;

    fp64 MaxError = 0.0;
    fp64 SumError = 0.0;
    u64 NumErrors = 0;
    u64 NumAboveThreshold = 0;

    std::vector<u64> CulledMapPointIDs;
    std::vector<std::vector<u64>> AllCulledIndexes;
    // Map-point IDs index the backing storage, whose size can exceed the
    // active count after removals leave sparse slots.
    AllCulledIndexes.resize(GlobalMap->MapPoints.size());
    CulledMapPointIDs.reserve(InitialMapPointCount);

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Starting with %zu active map points\n",
            InitialMapPointCount);

    for(typePantoMapPoint& MapPoint : GlobalMap->MapPoints)
    {
        typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;
        typePantoVector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;

        const u64 MapPointID = MapPoint.ID;

        assert(KeyFrameIDs.size() == ImagePointIDs.size());
        const u64 NumObservationsBefore = PT_GetNumObservations(MapPoint);
        Eigen::Vector2d ProjectedPoint;
        std::vector<u64> CulledIndexes;
        CulledIndexes.reserve(MapPoint.KeyFrameIDs.active_size());

        for(std::size_t i{}; i < KeyFrameIDs.size(); i++)
        {
            assert(KeyFrameIDs.contains(i) == ImagePointIDs.contains(i));

            if(!KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = KeyFrameIDs[i];
            const u64 ImagePointID = ImagePointIDs[i];
            assert(GlobalMap->KeyFrames.contains( KeyFrameID));
            typeKeyFrame& KeyFrame = GlobalMap->KeyFrames[KeyFrameID];
            assert(KeyFrame.Points.ImagePoints.contains( ImagePointID));
            typePantoImagePoint& ImagePoint = KeyFrame.Points.ImagePoints[ImagePointID];
            if(!PROJ_Project(MapPoint.Point, ProjectedPoint, KeyFrame.Camera))
            {
                LG_Log( LogSeverity::DBG,
                        "[MAP_CullObservationEdges] MP %llu observation KF %llu IP %llu rejected: projection failed\n",
                        MapPoint.ID,
                        KeyFrameID,
                        ImagePointID);

                CulledIndexes.push_back(i);
                MappingData.NumObservationEdgesFailedProjection++;
                MappingData.ObservationEdgesCulled++;
                continue;
            }

            const fp64 PixelError = PROJ_PixelDistance( ProjectedPoint, ImagePoint.Point);
            if(PixelError > MaxError)
            {
                MaxError = PixelError;
            }
            NumErrors++;
            SumError+=PixelError;

            if(PixelError > PANTO_PIXEL_CHI_SQUARED_T_SQRT)
            {
                LG_Log( LogSeverity::DBG,
                        "[MAP_CullObservationEdges] MP %llu observation KF %llu IP %llu rejected: error = %.6f > %.6f\n",
                        MapPoint.ID,
                        KeyFrameID,
                        ImagePointID,
                        PixelError,
                        PANTO_PIXEL_CHI_SQUARED_T_SQRT);


                CulledIndexes.push_back(i);
                NumAboveThreshold++;
                MappingData.ObservationEdgesCulled++;
                MappingData.NumObservationEdgesPixelErrorHigh++;
                MappingData.SumPixelErrorRemovedPixels += PixelError;
                MappingData.SquaredSumPixelErrorRemovedPixels += PixelError*PixelError;
            }
        }

        if(!CulledIndexes.empty())
        {
            NumMapPointsWithBadEdges++;
        }

        const u64 RemainingObservations = NumObservationsBefore - static_cast<u64>( CulledIndexes.size());

        if(!CulledIndexes.empty())
        {
            LG_Log( LogSeverity::DBG,
                    "[MAP_CullObservationEdges] MP %llu: observations %llu -> %llu, bad edges = %zu\n",
                    MapPoint.ID,
                    NumObservationsBefore,
                    RemainingObservations,
                    CulledIndexes.size());
        }

        if(RemainingObservations < 2)
        {
            LG_Log( LogSeverity::DBG, "[MAP_CullObservationEdges] MP %llu marked for full deletion: remaining observations = %llu\n",
                    MapPoint.ID, RemainingObservations);
            CulledMapPointIDs.push_back( MapPoint.ID);
            MappingData.MapPointsCulled++;

            continue;
        }

        AllCulledIndexes[MapPointID] = CulledIndexes;

    }

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] MapPoints with bad edges = %llu\n",
            NumMapPointsWithBadEdges);

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Removing %zu entire map points\n",
            CulledMapPointIDs.size());

    for(std::size_t i{}; i < AllCulledIndexes.size(); i++)
    {
        const std::vector<u64>& CulledIndexes = AllCulledIndexes[i];
        if(CulledIndexes.empty())
        {
            continue;
        }

        typePantoMapPoint& MapPoint = GlobalMap->MapPoints[i];
        typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;
        typePantoVector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;

        for(const u64 CulledIndex : CulledIndexes)
        {
            assert(KeyFrameIDs.contains(CulledIndex));
            assert(ImagePointIDs.contains(CulledIndex));

            const u64 KeyFrameID = KeyFrameIDs[CulledIndex];
            const u64 ImagePointID = ImagePointIDs[CulledIndex];

            typePantoImagePoint& ImagePoint = GlobalMap->KeyFrames[ KeyFrameID]. Points.ImagePoints[ ImagePointID];
            GRAPH_DecrementAllOther( CovisibilityGraph, KeyFrameIDs, CulledIndex);

            ImagePoint.MapPointID = PANTO_ID_NOT_SET;
            KeyFrameIDs.remove(CulledIndex);
            ImagePointIDs.remove(CulledIndex);
            NumCulledObservationEdges++;
        }
    }
    for(const u64 CulledMapPointID : CulledMapPointIDs)
    {
        assert( GlobalMap->MapPoints.contains( CulledMapPointID));
        typePantoMapPoint& MapPoint = GlobalMap->MapPoints[ CulledMapPointID];
        typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;
        typePantoVector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;
        assert( KeyFrameIDs.size() == ImagePointIDs.size());
        LG_Log( LogSeverity::DBG,
                "[MAP_CullObservationEdges] Fully removing MP %llu with %llu observations\n",
                CulledMapPointID,
                PT_GetNumObservations(MapPoint));

        GRAPH_DecrementAll(CovisibilityGraph, KeyFrameIDs);

        for(std::size_t i{}; i < KeyFrameIDs.size(); i++)
        {
            assert( KeyFrameIDs.contains(i) == ImagePointIDs.contains(i));

            if(!KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = KeyFrameIDs[i];
            const u64 ImagePointID = ImagePointIDs[i];
            assert( GlobalMap->KeyFrames.contains(
                            KeyFrameID));
            assert(GlobalMap->KeyFrames[ KeyFrameID]. Points.ImagePoints.contains(
                            ImagePointID));
            typePantoImagePoint& ImagePoint = GlobalMap->KeyFrames[ KeyFrameID].
                    Points.ImagePoints[
                        ImagePointID];
            ImagePoint.MapPointID = PANTO_ID_NOT_SET;
        }
        GlobalMap->MapPoints.remove( CulledMapPointID);
    }

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Removed %llu observation edges\n",
            NumCulledObservationEdges);

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] MapPoints: %zu -> %zu\n",
            InitialMapPointCount,
            GlobalMap->MapPoints.active_size());

    LG_Log(LogSeverity::DBG, 
            "[MAP_CullObservationEdges] Mean pixel error: %lf, Max: %lf, NumAbove: %llu\n",
            SumError / static_cast<fp64>(NumErrors),
            MaxError,
            NumAboveThreshold
          );
}

std::vector<u64> MAP_CreateNewMapPoints(typeGlobalMap* GlobalMap, typeKeyFrame& NewKeyFrame, typeCovisibilityGraph* CovisibilityGraph,
        const u64 LatestKeyFrameID)
{
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(*CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);
    std::vector<typeKeyFrame> LocalMapKeyFrames;

    for(const typeCovisibility& Covisibility : MostCovisible)
    {
        LocalMapKeyFrames.push_back(GlobalMap->KeyFrames[Covisibility.KeyFrameID]);
    }

    std::unordered_set<u64> LocalMapPointIDs;

    for(const typeKeyFrame& KeyFrame : LocalMapKeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint :
                GlobalMap->KeyFrames[KeyFrame.ID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }

            LocalMapPointIDs.insert( ImagePoint.MapPointID);
        }
    }

    std::vector<typePantoMapPoint> LocalMapMapPoints;
    LocalMapMapPoints.reserve(LocalMapPointIDs.size());

    for(const u64 MapPointID : LocalMapPointIDs)
    {
        if(GlobalMap->MapPoints.contains(MapPointID))
        {
            LocalMapMapPoints.push_back(GlobalMap->MapPoints[MapPointID]);
        }
    }

    Eigen::Vector3d NewCameraCenter = CM_GetCameraCenter(NewKeyFrame.Camera);
    std::vector<u64> NewPointIDs;
    NewPointIDs.reserve(PANTO_NEW_MAPPOINT_RESERVE * LocalMapKeyFrames.size());

    for(const typeKeyFrame& KeyFrameLocal : LocalMapKeyFrames)
    {
        // Ignore new keyframe
        if(KeyFrameLocal.ID == NewKeyFrame.ID)
        {
            continue;
        }

        const Eigen::Vector3d CameraCenter = CM_GetCameraCenter(KeyFrameLocal.Camera);

        const fp64 BaseLine = (NewCameraCenter - CameraCenter).norm();

        const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(KeyFrameLocal, LocalMapMapPoints);

        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Median Depth in local map = %lf\n", MedianDepth); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline in between keyframes = %lf\n", BaseLine); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline is large enough = %d \n", PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth)); 

        typeKeyFrame& KeyFrame = GlobalMap->KeyFrames[KeyFrameLocal.ID];

        if(PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth))
        {
            const std::vector<typePantoMapPoint> MapPoints = KEY_InsertNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap->Age);
            // Commit this pair before considering another covisible frame.
            // This marks NewKeyFrame's used image points and prevents them
            // from being triangulated into multiple map points across pairs.
            for(const typePantoMapPoint& MapPoint : MapPoints)
            {
                const std::vector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;
                const std::vector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;
                typeKeyFrame& KeyFrame1 = GlobalMap->KeyFrames[KeyFrameIDs[0]];
                typeKeyFrame& KeyFrame2 = GlobalMap->KeyFrames[KeyFrameIDs[1]];

                typePantoImagePoint& ImagePoint1 =
                    KeyFrame1.Points.ImagePoints[ImagePointIDs[0]];
                typePantoImagePoint& ImagePoint2 =
                    KeyFrame2.Points.ImagePoints[ImagePointIDs[1]];

                assert(ImagePoint1.MapPointID == PANTO_ID_NOT_SET);
                assert(ImagePoint2.MapPointID == PANTO_ID_NOT_SET);

                const u64 MapPointID = GlobalMap->MapPoints.push_back(MapPoint);
                GlobalMap->MapPoints[MapPointID].ID = MapPointID;
                ImagePoint1.MapPointID = MapPointID;
                ImagePoint2.MapPointID = MapPointID;
                NewPointIDs.push_back(MapPointID);
            }
        }
    }

    GRAPH_UpdateCovisibility(CovisibilityGraph, GlobalMap->MapPoints, LatestKeyFrameID, NewPointIDs);

    return NewPointIDs;
}

std::vector<u64> MAP_FuseMapPoints(typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph, typeKeyFrame& NewKeyFrame)
{
    std::vector<typeCovisibility> Top30Neighbours = GRAPH_GetTopNCovisibleFrames(*CovisibilityGraph, NewKeyFrame.ID, PANTO_MAPPOINT_FUSION_N_FIRST_ORDER_NEIGHBOURS);
    std::vector<typeCovisibility> EligibleNeighbours;
    EligibleNeighbours.insert(EligibleNeighbours.end(), Top30Neighbours.begin(), Top30Neighbours.end());

    u64 NumberOfCandidates = Top30Neighbours.size();
    u64 i = 0;
    while(NumberOfCandidates < 20 && i < static_cast<u64>(Top30Neighbours.size()))
    {
        const u64 KeyFrameID = Top30Neighbours[i].KeyFrameID;
        std::vector<typeCovisibility> Top20SecondOrderNeighbours = GRAPH_GetTopNCovisibleFrames(*CovisibilityGraph, KeyFrameID, 20);
        EligibleNeighbours.insert(EligibleNeighbours.end(), Top20SecondOrderNeighbours.begin(), Top20SecondOrderNeighbours.end());
        NumberOfCandidates += static_cast<u64>(Top20SecondOrderNeighbours.size());
        i++;
    }

    std::vector<typePantoMapPoint> LatestFrameMapPoints;
    for(const typePantoImagePoint& ImagePoint : NewKeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID != PANTO_ID_NOT_SET)
        {
            assert(GlobalMap->MapPoints.contains(MapPointID));
            LatestFrameMapPoints.push_back(GlobalMap->MapPoints[MapPointID]);
        }
    }

    // Contains the indexes of mappoints absorbed into a different mappoint
    std::vector<u64> FusedMapPoints;
    FusedMapPoints.reserve(LatestFrameMapPoints.size());
    std::unordered_set<u64> FusedMapPointIDs;

    const auto HasObservation = [](const typePantoMapPoint& MapPoint,
            const u64 KeyFrameID)
    {
        for(const u64 ObservingKeyFrameID : MapPoint.KeyFrameIDs)
        {
            if(ObservingKeyFrameID == KeyFrameID)
            {
                return true;
            }
        }
        return false;
    };

    const auto RecalculateDescriptor = [GlobalMap](typePantoMapPoint& MapPoint)
    {
        std::vector<typeDescriptor> Descriptors;
        Descriptors.reserve(MapPoint.KeyFrameIDs.active_size());
        for(std::size_t ObservationIndex = 0;
            ObservationIndex < MapPoint.KeyFrameIDs.size();
            ObservationIndex++)
        {
            if(!MapPoint.KeyFrameIDs.contains(ObservationIndex))
            {
                continue;
            }
            const u64 KeyFrameID = MapPoint.KeyFrameIDs[ObservationIndex];
            const u64 ImagePointID = MapPoint.ImagePointIDs[ObservationIndex];
            Descriptors.push_back(GlobalMap->KeyFrames[KeyFrameID].
                    Points.ImagePoints[ImagePointID].Descriptor);
        }
        if(!Descriptors.empty())
        {
            MapPoint.Descriptor = Descriptors.size() == 1
                ? Descriptors.front()
                : PT_CalculateNewDescriptor(Descriptors);
        }
    };

    for(const typePantoMapPoint& MapPoint : LatestFrameMapPoints)
    {
        if(FusedMapPointIDs.contains(MapPoint.ID) ||
           !GlobalMap->MapPoints.contains(MapPoint.ID))
        {
            continue;
        }

        std::vector<typeDescriptor> Descriptors;
        Descriptors.reserve(MapPoint.ImagePointIDs.active_size() + 1);
        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            if(MapPoint.KeyFrameIDs.contains(i))
            {
                assert(MapPoint.ImagePointIDs.contains(i));
                const u64 KFID = MapPoint.KeyFrameIDs[i];
                const u64 ImagePointID = MapPoint.ImagePointIDs[i];
                Descriptors.push_back(GlobalMap->KeyFrames[KFID].Points.ImagePoints[ImagePointID].Descriptor);
            }
        }
        for(const typeCovisibility& Neighbour : EligibleNeighbours)
        {
            typeKeyFrame& NeighbourKeyFrame = GlobalMap->KeyFrames[Neighbour.KeyFrameID];
            typePantoMapPoint& LiveMapPoint =
                GlobalMap->MapPoints[MapPoint.ID];
            if(HasObservation(LiveMapPoint, NeighbourKeyFrame.ID))
            {
                // Already observed
                continue;
            }

            const typeCamera NeighbourPose = NeighbourKeyFrame.Camera;
            Eigen::Vector2d ProjectedPoint;
            if(PROJ_Project(MapPoint.Point, ProjectedPoint, NeighbourPose))
            {
                if(!ProjectedPoint.allFinite() ||
                   ProjectedPoint.x() < 0.0 ||
                   ProjectedPoint.y() < 0.0 ||
                   ProjectedPoint.x() >= static_cast<fp64>(PANTO_IMAGE_WIDTH) ||
                   ProjectedPoint.y() >= static_cast<fp64>(PANTO_IMAGE_HEIGHT))
                {
                    continue;
                }

                const u64 Truncatedx = static_cast<u64>(ProjectedPoint.x());
                const u64 Truncatedy = static_cast<u64>(ProjectedPoint.y());
                const u64 CellX = (Truncatedx) / PANTO_CELL_SIZE;
                const u64 CellY = (Truncatedy) / PANTO_CELL_SIZE;

                const u64 CellIndex = CellY * PANTO_GRID_COLUMNS + CellX;
                std::vector<u64> EligibleImagePointIDs = NeighbourKeyFrame.Points.CellIndexingArray[CellIndex];
                const typeDescriptor MapPointDescriptor = MapPoint.Descriptor;

                const typePantoVector<typePantoImagePoint> ImagePoints = NeighbourKeyFrame.Points.ImagePoints;
                u32 ShortestHamming = std::numeric_limits<u32>::max();
                u64 ClosestPointIndex = PANTO_ID_NOT_SET;
                typeDescriptor ClosestDescriptor = typeDescriptor{};

                for(const u64 PointIndex : EligibleImagePointIDs)
                {
                    const u32 HammingDistance = PANTO_HammingDistance(MapPointDescriptor, ImagePoints[PointIndex].Descriptor);
                    if(HammingDistance < ShortestHamming)
                    {
                        ShortestHamming = HammingDistance;
                        ClosestPointIndex = PointIndex;
                        ClosestDescriptor = ImagePoints[PointIndex].Descriptor;
                    }
                }

                if(ShortestHamming < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW)
                {
                    //TODO update covisibility graph too! 
                    const u64 AssociatedMapPointID = ImagePoints[ClosestPointIndex].MapPointID;
                    if(AssociatedMapPointID != PANTO_ID_NOT_SET)
                    {
                        if(AssociatedMapPointID == MapPoint.ID ||
                           !GlobalMap->MapPoints.contains(AssociatedMapPointID))
                        {
                            continue;
                        }

                        const u64 NumVisibleLatest = static_cast<u64>(MapPoint.KeyFrameIDs.active_size());
                        const u64 NumVisibleHistoric = static_cast<u64>(GlobalMap->MapPoints[AssociatedMapPointID].KeyFrameIDs.active_size());
                        if(NumVisibleLatest > NumVisibleHistoric)
                        {
                            // Fuse historic into latest
                            typePantoMapPoint& MapPointMutable = GlobalMap->MapPoints[MapPoint.ID];
                            const typePantoMapPoint& HistoricMapPoint = GlobalMap->MapPoints[AssociatedMapPointID];
                            GRAPH_DecrementAll(CovisibilityGraph,
                                    HistoricMapPoint.KeyFrameIDs);
                            for(std::size_t i{}; i < HistoricMapPoint.KeyFrameIDs.size(); i++)
                            {
                                if(HistoricMapPoint.KeyFrameIDs.contains(i))
                                {
                                    assert(HistoricMapPoint.ImagePointIDs.contains(i));
                                    const u64 KeyFrameID = HistoricMapPoint.KeyFrameIDs[i];
                                    const u64 ImagePointID = HistoricMapPoint.ImagePointIDs[i];
                                    if(HasObservation(MapPointMutable, KeyFrameID))
                                    {
                                        GlobalMap->KeyFrames[KeyFrameID].Points.
                                            ImagePoints[ImagePointID].MapPointID =
                                            PANTO_ID_NOT_SET;
                                        continue;
                                    }
                                    for(const u64 ExistingKeyFrameID :
                                            MapPointMutable.KeyFrameIDs)
                                    {
                                        GRAPH_IncrementEdge(CovisibilityGraph,
                                                ExistingKeyFrameID,
                                                KeyFrameID);
                                    }
                                    MapPointMutable.KeyFrameIDs.push_back(KeyFrameID);
                                    MapPointMutable.ImagePointIDs.push_back(ImagePointID);
                                    GlobalMap->KeyFrames[KeyFrameID].Points.ImagePoints[ImagePointID].MapPointID = MapPointMutable.ID;
                                }
                            }
                            if(FusedMapPointIDs.insert(HistoricMapPoint.ID).second)
                            {
                                FusedMapPoints.push_back(HistoricMapPoint.ID);
                                MappingData.MapPointFusions++;
                            }
                            RecalculateDescriptor(MapPointMutable);
                        }
                        else
                        {
                            // Fuse latest into historic 
                            const typePantoMapPoint& MapPointNonMutable = GlobalMap->MapPoints[MapPoint.ID];
                            typePantoMapPoint& HistoricMapPoint = GlobalMap->MapPoints[AssociatedMapPointID];
                            GRAPH_DecrementAll(CovisibilityGraph,
                                    MapPointNonMutable.KeyFrameIDs);
                            for(std::size_t i{}; i < MapPointNonMutable.KeyFrameIDs.size(); i++)
                            {
                                if(MapPointNonMutable.KeyFrameIDs.contains(i))
                                {
                                    assert(MapPointNonMutable.ImagePointIDs.contains(i));
                                    const u64 KeyFrameID = MapPointNonMutable.KeyFrameIDs[i];
                                    const u64 ImagePointID = MapPointNonMutable.ImagePointIDs[i];
                                    if(HasObservation(HistoricMapPoint, KeyFrameID))
                                    {
                                        GlobalMap->KeyFrames[KeyFrameID].Points.
                                            ImagePoints[ImagePointID].MapPointID =
                                            PANTO_ID_NOT_SET;
                                        continue;
                                    }
                                    for(const u64 ExistingKeyFrameID :
                                            HistoricMapPoint.KeyFrameIDs)
                                    {
                                        GRAPH_IncrementEdge(CovisibilityGraph,
                                                ExistingKeyFrameID,
                                                KeyFrameID);
                                    }
                                    HistoricMapPoint.KeyFrameIDs.push_back(KeyFrameID);
                                    HistoricMapPoint.ImagePointIDs.push_back(ImagePointID);
                                    GlobalMap->KeyFrames[KeyFrameID].Points.ImagePoints[ImagePointID].MapPointID = HistoricMapPoint.ID;
                                }
                            }
                            if(FusedMapPointIDs.insert(MapPointNonMutable.ID).second)
                            {
                                FusedMapPoints.push_back(MapPointNonMutable.ID);
                                MappingData.MapPointFusions++;
                            }
                            RecalculateDescriptor(HistoricMapPoint);
                        }
                    }
                    else
                    {
                        Descriptors.push_back(ClosestDescriptor);
                        typePantoMapPoint& MapPointMutable = GlobalMap->MapPoints[MapPoint.ID];
                        MapPointMutable.Descriptor = PT_CalculateNewDescriptor(Descriptors);
                        for(const u64 ExistingKeyFrameID :
                                MapPointMutable.KeyFrameIDs)
                        {
                            GRAPH_IncrementEdge(CovisibilityGraph,
                                    ExistingKeyFrameID,
                                    NeighbourKeyFrame.ID);
                        }
                        MapPointMutable.KeyFrameIDs.push_back(NeighbourKeyFrame.ID);
                        MapPointMutable.ImagePointIDs.push_back(ClosestPointIndex);
                        MapPointMutable.NumFound++;
                        MapPointMutable.NumVisible++;
                        MappingData.MapPointFusionObservations++;
                        NeighbourKeyFrame.Points.ImagePoints[ClosestPointIndex].MapPointID = MapPointMutable.ID;
                    }
                    break;
                }
            }
        }
    }

    for(const u64 FusedMapPointID : FusedMapPoints)
    {
        if(GlobalMap->MapPoints.contains(FusedMapPointID))
        {
            GlobalMap->MapPoints.remove(FusedMapPointID);
        }
    }
    return FusedMapPoints;
}

void MAP_LogGlobalMapPoses(const typeGlobalMap& GlobalMap)
{
    LG_Log(
        LogSeverity::DBG,
        "[MAP_LogGlobalMapPoses] Logging %zu keyframe poses\n",
        GlobalMap.KeyFrames.size());

    for(const typeKeyFrame& KeyFrame : GlobalMap.KeyFrames)
    {
        const typeCamera& Camera = KeyFrame.Camera;

        LG_Log(
            LogSeverity::DBG,
            "[MAP_LogGlobalMapPoses] KeyFrame %llu q = (%f, %f, %f, %f), t = (%f, %f, %f)\n",
            static_cast<unsigned long long>(KeyFrame.ID),
            Camera.Pose.Quaternion.w(),
            Camera.Pose.Quaternion.x(),
            Camera.Pose.Quaternion.y(),
            Camera.Pose.Quaternion.z(),
            Camera.Pose.tParametrization[0],
            Camera.Pose.tParametrization[1],
            Camera.Pose.tParametrization[2]);
    }
}


void MAP_LogKeyFrameProjectionError(const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints)
{
    std::vector<fp64> Errors;
    Errors.reserve(KeyFrame.Points.ImagePoints.size());

    u64 Below2 = 0;
    u64 Below5 = 0;
    u64 Below10 = 0;
    u64 Below20 = 0;
    u64 Above20 = 0;
    u64 NumFailedProjection = 0;

    for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        assert(ImagePoint.MapPointID < GlobalMapPoints.size());

        const typePantoMapPoint& MapPoint = GlobalMapPoints[ImagePoint.MapPointID];

        Eigen::Vector2d ProjectedPoint{};

        if(!PROJ_Project(MapPoint.Point, ProjectedPoint, KeyFrame.Camera))
        {
            NumFailedProjection++;
            continue;
        }

        const fp64 Error = (ImagePoint.Point - ProjectedPoint).norm();

        if(!std::isfinite(Error))
        {
            NumFailedProjection++;
            continue;
        }

        Errors.push_back(Error);

        if(Error < 2.0)
        {
            Below2++;
        }
        else if(Error < 5.0)
        {
            Below5++;
        }
        else if(Error < 10.0)
        {
            Below10++;
        }
        else if(Error < 20.0)
        {
            Below20++;
        }
        else
        {
            Above20++;
        }
    }

    if(Errors.empty())
    {
        LG_Log(
            LogSeverity::DBG,
            "[MAP_LogKeyFrameProjectionError] KeyFrame %llu has no valid projection errors\n",
            static_cast<unsigned long long>(KeyFrame.ID));

        return;
    }

    fp64 Mean = 0.0;

    for(const fp64 Error : Errors)
    {
        Mean += Error;
    }

    Mean /= static_cast<fp64>(Errors.size());

    fp64 Variance = 0.0;

    for(const fp64 Error : Errors)
    {
        const fp64 Difference = Error - Mean;
        Variance += Difference * Difference;
    }

    Variance /= static_cast<fp64>(Errors.size());

    const fp64 StandardDeviation = std::sqrt(Variance);

    LG_Log(
        LogSeverity::DBG,
        "[MAP_LogKeyFrameProjectionError] KeyFrame %llu: %zu valid projections, %llu failed\n",
        static_cast<unsigned long long>(KeyFrame.ID),
        Errors.size(),
        static_cast<unsigned long long>(NumFailedProjection));

    LG_Log(
        LogSeverity::DBG,
        "[MAP_LogKeyFrameProjectionError] Mean = %f px, std dev = %f px\n",
        Mean,
        StandardDeviation);

    LG_Log(
        LogSeverity::DBG,
        "[MAP_LogKeyFrameProjectionError] Distribution: <2px = %llu, 2-5px = %llu, 5-10px = %llu, 10-20px = %llu, >=20px = %llu\n",
        static_cast<unsigned long long>(Below2),
        static_cast<unsigned long long>(Below5),
        static_cast<unsigned long long>(Below10),
        static_cast<unsigned long long>(Below20),
        static_cast<unsigned long long>(Above20));
}

void MAP_LogGlobalMapProjectionErrors(const typeGlobalMap& GlobalMap)
{
    LG_Log(
        LogSeverity::DBG,
        "[MAP_LogGlobalMapProjectionErrors] Logging projection errors for %zu keyframes\n",
        GlobalMap.KeyFrames.size());

    for(const typeKeyFrame& KeyFrame : GlobalMap.KeyFrames)
    {
        MAP_LogKeyFrameProjectionError(
            KeyFrame,
            GlobalMap.MapPoints);
    }
}

void MAP_RetriangulateLOST(typeGlobalMap& GlobalMap)
{
    std::vector<std::vector<Eigen::Vector3d>> PixelCoords;
    std::vector<std::vector<Eigen::Matrix4d>> Transforms;
    std::vector<u64> MapPointIDs;

    PixelCoords.reserve(GlobalMap.MapPoints.size());
    Transforms.reserve(GlobalMap.MapPoints.size());
    MapPointIDs.reserve(GlobalMap.MapPoints.size());

    for(typePantoMapPoint& MapPoint : GlobalMap.MapPoints)
    {
        assert(MapPoint.KeyFrameIDs.size() ==
                MapPoint.ImagePointIDs.size());

        if(MapPoint.KeyFrameIDs.size() < 2)
        {
            continue;
        }

        std::vector<Eigen::Vector3d> MapPointPixelCoords;
        std::vector<Eigen::Matrix4d> MapPointTransforms;

        MapPointPixelCoords.reserve(MapPoint.KeyFrameIDs.size());
        MapPointTransforms.reserve(MapPoint.KeyFrameIDs.size());

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            const u64 KeyFrameID = MapPoint.KeyFrameIDs[i];
            const u64 ImagePointID = MapPoint.ImagePointIDs[i];

            const typeKeyFrame& KeyFrame =
                GlobalMap.KeyFrames[KeyFrameID];

            const typePantoImagePoint& ImagePoint =
                KeyFrame.Points.ImagePoints[ImagePointID];

            Eigen::Vector3d PixelCoord
            {
                ImagePoint.Point.x(),
                ImagePoint.Point.y(),
                1.0
            };

            Eigen::Matrix4d Transform =
                Eigen::Matrix4d::Identity();

            Transform.block<3, 3>(0, 0) =
                KeyFrame.Camera.Pose.R;

            Transform.block<3, 1>(0, 3) =
                KeyFrame.Camera.Pose.t;

            MapPointPixelCoords.push_back(PixelCoord);
            MapPointTransforms.push_back(Transform);
        }

        PixelCoords.push_back(std::move(MapPointPixelCoords));
        Transforms.push_back(std::move(MapPointTransforms));
        MapPointIDs.push_back(MapPoint.ID);
    }

    if(PixelCoords.empty())
    {
        return;
    }

    assert(!GlobalMap.KeyFrames.empty());
    assert(GlobalMap.KeyFrames[0].Camera.Intrinsics != nullptr);

    const Eigen::Matrix3d K =
        GlobalMap.KeyFrames[0].Camera.Intrinsics->K;

    const std::vector<Eigen::Vector4d> RetriangulatedPoints =
        PROJ_TriangulateLOST(
                PixelCoords,
                Transforms,
                K);

    assert(RetriangulatedPoints.size() ==
            MapPointIDs.size());

    for(std::size_t i{}; i < RetriangulatedPoints.size(); i++)
    {
        GlobalMap.MapPoints[MapPointIDs[i]].Point =
            PROJ_NormalizeToSpherical(
                    RetriangulatedPoints[i]);
    }

    LG_Log(LogSeverity::DBG,
            "[MAP_RetriangulateLOST] Retriangulated %llu map points\n",
            static_cast<unsigned long long>(RetriangulatedPoints.size()));
}

void MAP_AssertGraphEqual(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph)
{
    assert(GlobalMap.KeyFrames.size() == CovisibilityGraph.CovisibilityGraph.size());

    std::vector<std::unordered_map<u64, u64>> ExpectedConnections(
            GlobalMap.KeyFrames.size());

    for(std::size_t i{}; i < GlobalMap.KeyFrames.size(); i++)
    {
        assert(GlobalMap.KeyFrames.contains(i) == CovisibilityGraph.CovisibilityGraph.contains(i));

        if(GlobalMap.KeyFrames.contains(i))
        {
            assert(GlobalMap.KeyFrames[i].ID == i);
        }
    }

    for(const typePantoMapPoint& MapPoint : GlobalMap.MapPoints)
    {
        const typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;

        for(std::size_t i = 0; i < KeyFrameIDs.size(); i++)
        {
            if(!KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 FirstKeyFrameID = KeyFrameIDs[i];
            assert(GlobalMap.KeyFrames.contains(FirstKeyFrameID));

            for(std::size_t j = i + 1; j < KeyFrameIDs.size(); j++)
            {
                if(!KeyFrameIDs.contains(j))
                {
                    continue;
                }

                const u64 SecondKeyFrameID = KeyFrameIDs[j];
                assert(FirstKeyFrameID != SecondKeyFrameID);
                assert(GlobalMap.KeyFrames.contains(SecondKeyFrameID));

                ExpectedConnections[FirstKeyFrameID][SecondKeyFrameID]++;
                ExpectedConnections[SecondKeyFrameID][FirstKeyFrameID]++;
            }
        }
    }

    for(std::size_t KeyFrameID = 0;
        KeyFrameID < GlobalMap.KeyFrames.size();
        KeyFrameID++)
    {
        if(!GlobalMap.KeyFrames.contains(KeyFrameID))
        {
            continue;
        }

        const auto& ActualConnections =
            CovisibilityGraph.CovisibilityGraph[KeyFrameID];
        const auto& ExpectedKeyFrameConnections =
            ExpectedConnections[KeyFrameID];

        if(ActualConnections != ExpectedKeyFrameConnections)
        {
            LG_Log(LogSeverity::ERROR,
                    "[MAP_AssertGraphEqual] KeyFrame %llu graph mismatch; actual edges = %zu, expected edges = %zu\n",
                    static_cast<unsigned long long>(KeyFrameID),
                    ActualConnections.size(),
                    ExpectedKeyFrameConnections.size());

            for(const auto& [OtherKeyFrameID, ExpectedCount] :
                    ExpectedKeyFrameConnections)
            {
                const auto ActualIt = ActualConnections.find(OtherKeyFrameID);
                const u64 ActualCount = ActualIt == ActualConnections.end()
                    ? 0
                    : ActualIt->second;
                if(ActualCount != ExpectedCount)
                {
                    LG_Log(LogSeverity::ERROR,
                            "[MAP_AssertGraphEqual] Edge %llu -> %llu: actual = %llu, expected = %llu\n",
                            static_cast<unsigned long long>(KeyFrameID),
                            static_cast<unsigned long long>(OtherKeyFrameID),
                            static_cast<unsigned long long>(ActualCount),
                            static_cast<unsigned long long>(ExpectedCount));
                }
            }

            for(const auto& [OtherKeyFrameID, ActualCount] : ActualConnections)
            {
                if(!ExpectedKeyFrameConnections.contains(OtherKeyFrameID))
                {
                    LG_Log(LogSeverity::ERROR,
                            "[MAP_AssertGraphEqual] Unexpected edge %llu -> %llu: actual = %llu, expected = 0\n",
                            static_cast<unsigned long long>(KeyFrameID),
                            static_cast<unsigned long long>(OtherKeyFrameID),
                            static_cast<unsigned long long>(ActualCount));
                }
            }

            assert(false);
        }
    }
}

void MAP_AssertMapPointObservations( const typeGlobalMap& GlobalMap)
{
    for(const typePantoMapPoint& MapPoint : GlobalMap.MapPoints)
    {
        assert( MapPoint.KeyFrameIDs.size() == MapPoint.ImagePointIDs.size());

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            assert( MapPoint.KeyFrameIDs.contains(i) == MapPoint.ImagePointIDs.contains(i));

            if(!MapPoint.KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = MapPoint.KeyFrameIDs[i];

            const u64 ImagePointID = MapPoint.ImagePointIDs[i];

            if(!GlobalMap.KeyFrames.contains(KeyFrameID))
            {
                LG_Log( LogSeverity::ERROR,
                        "[MAP_AssertMapPointObservations] MP %llu references removed KF %llu\n",
                        MapPoint.ID,
                        KeyFrameID);

                assert(false);
            }

            const typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameID];

            assert( KeyFrame.Points.ImagePoints.contains( ImagePointID));

            const typePantoImagePoint& ImagePoint = KeyFrame.Points.ImagePoints[ImagePointID];

            if(ImagePoint.MapPointID != MapPoint.ID)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_AssertMapPointObservations] MP %llu -> KF %llu IP %llu, but IP points to MP %llu\n",
                        MapPoint.ID,
                        KeyFrameID,
                        ImagePointID,
                        ImagePoint.MapPointID);

                assert(false);
            }
        }
    }
}

u64 MAP_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame,
        std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose,
        u64* NumProjectedMapPointsOutput,
        std::vector<u64>* VisibleMapPointIDsOutput,
        std::vector<u64>* FoundMapPointIDsOutput)
{
    std::unordered_set<u64> UniqueMapPointIDs;

#if defined(DEBUG)
    for(const typePantoMapPoint& MapPoint : MapPoints)
    {
        assert(UniqueMapPointIDs.insert(MapPoint.ID).second);
    }
#endif

    std::unordered_set<u64> AssociatedMapPointIDs;

    for(const typePantoImagePoint& ImagePoint : KeyFrame.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
        {
            AssociatedMapPointIDs.insert(ImagePoint.MapPointID);
        }
    }

    std::size_t NumMapPoints = MapPoints.size();

    u64 NumTrackedMapPoints = 0;
    u64 NumNewMatchedMapPoints = 0;

    u64 NumProjectedMapPoints = 0;
    u64 NumCandidateImagePoints = 0;
    u64 NumWithTwoCandidates = 0;

    // These map points are tracking snapshots. Only collect observation IDs
    // when the caller will commit them to the global map; otherwise updating
    // visibility/found bookkeeping here would be discarded with the snapshot.
    const bool CollectVisibleMapPoints = VisibleMapPointIDsOutput != nullptr;
    const bool CollectFoundMapPoints = FoundMapPointIDsOutput != nullptr;
    std::vector<u64> VisibleMapPoints;
    std::vector<u64> FoundMapPoints;
    if(CollectVisibleMapPoints)
        VisibleMapPoints.reserve(NumMapPoints);
    if(CollectFoundMapPoints)
        FoundMapPoints.reserve(NumMapPoints);

    for(std::size_t i{}; i < NumMapPoints; i++)
    {
        const typePantoMapPoint& MapPoint = MapPoints[i];
        Eigen::Vector2d CandidateImagePoint = {};
        const u64 MapPointID = MapPoint.ID;

        const bool AlreadyAssociated = AssociatedMapPointIDs.contains(MapPointID);

        if(PROJ_Project(MapPoint.Point, CandidateImagePoint, Pose))
        {
            NumProjectedMapPoints++;
            if(CollectVisibleMapPoints)
                VisibleMapPoints.push_back(MapPointID);

            if(AlreadyAssociated)
            {
                NumTrackedMapPoints++;
                continue;
            }

            const fp64 u = CandidateImagePoint[0];
            const fp64 v = CandidateImagePoint[1];

            const fp64 Radius = PANTO_MAPPOINT_MATCH_SEARCH_RADIUS;

            const fp64 MinU = u - Radius;
            const fp64 MaxU = u + Radius;
            const fp64 MinV = v - Radius;
            const fp64 MaxV = v + Radius;

            const i64 MinCellX = std::max<i64>( 0, static_cast<i64>((u - PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MaxCellX = std::min<i64>( PANTO_GRID_COLUMNS - 1, static_cast<i64>((u + PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MinCellY = std::max<i64>( 0, static_cast<i64>((v - PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MaxCellY = std::min<i64>( PANTO_GRID_ROWS - 1, static_cast<i64>((v + PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));

            const typeDescriptor& MapPointDescriptor = MapPoint.Descriptor;
            u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;

            u32 SecondBestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;

            u64 BestImagePointID = PANTO_ID_NOT_SET;

            for(i64 j(MinCellY); j <= MaxCellY; j++)
            {
                for(i64 k(MinCellX); k <= MaxCellX; k++)
                {
                    std::vector<u64>& LocalImagePoints = KeyFrame.CellIndexingArray[j * PANTO_GRID_COLUMNS + k];

                    for(const u64& ImagePointIdx : LocalImagePoints)
                    {
                        NumCandidateImagePoints++;

                        typePantoImagePoint& ImagePoint = KeyFrame.ImagePoints[ImagePointIdx];

                        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
                        {
                            continue;
                        }

                        const fp64 ImageU = ImagePoint.Point[0];
                        const fp64 ImageV = ImagePoint.Point[1];

                        if(
                                ImageU < MinU ||
                                ImageU > MaxU ||
                                ImageV < MinV ||
                                ImageV > MaxV)
                        {
                            continue;
                        }

                        const typeDescriptor& ImagePointDescriptor = ImagePoint.Descriptor;

                        const u32 HammingDistance = PANTO_HammingDistance( MapPointDescriptor, ImagePointDescriptor);

                        if(HammingDistance < BestDistance)
                        {
                            SecondBestDistance = BestDistance;

                            BestDistance = HammingDistance;
                            BestImagePointID = ImagePoint.ID;
                        }
                        else if(HammingDistance < SecondBestDistance)
                        {
                            SecondBestDistance = HammingDistance;
                        }
                    }
                }
            }
            NumWithTwoCandidates++;

            if(SecondBestDistance > PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
            {
                NumWithTwoCandidates--;
            }

            if(static_cast<fp64>(BestDistance) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestDistance)
                    && BestDistance < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
            {
                KeyFrame.ImagePoints[BestImagePointID].MapPointID = MapPointID;


                NumTrackedMapPoints++;
                NumNewMatchedMapPoints++;

                AssociatedMapPointIDs.insert(MapPointID);
                if(CollectFoundMapPoints)
                    FoundMapPoints.push_back(MapPointID);
            }
        }
    }

    if(NumProjectedMapPointsOutput != nullptr)
    {
        *NumProjectedMapPointsOutput = NumProjectedMapPoints;
    }
    if(VisibleMapPointIDsOutput != nullptr)
    {
        *VisibleMapPointIDsOutput = std::move(VisibleMapPoints);
    }
    if(FoundMapPointIDsOutput != nullptr)
    {
        *FoundMapPointIDsOutput = std::move(FoundMapPoints);
    }

    LG_Log(LogSeverity::DBG, "[PT_MatchMapPointsToKeyFrame] Projected %llu/%zu map points, checked %llu image points, %llu had two candidates, %llu tracked (%llu newly matched)\n",
        static_cast<unsigned long long>(NumProjectedMapPoints),
        NumMapPoints,
        static_cast<unsigned long long>(NumCandidateImagePoints),
        static_cast<unsigned long long>(NumWithTwoCandidates),
        static_cast<unsigned long long>(NumTrackedMapPoints),
        static_cast<unsigned long long>(NumNewMatchedMapPoints));

    return NumTrackedMapPoints;
}

void MAPPriv_CullRecentMapPoint(const u64 MapPointIndex, typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph)
{
    // Each map point contributes one unit to every pair of keyframes that
    // observes it. Remove those contributions before deleting observations.

    typePantoMapPoint& MapPoint = GlobalMap->MapPoints[MapPointIndex];
    GRAPH_DecrementAll(CovisibilityGraph, MapPoint.KeyFrameIDs);

    typePantoVector KeyFrameIDs = MapPoint.KeyFrameIDs;
    typePantoVector ImagePointIDs = MapPoint.ImagePointIDs;
    for(std::size_t j{}; j < KeyFrameIDs.size(); j++)
    {
        if(!KeyFrameIDs.contains(j) || !ImagePointIDs.contains(j)) 
        {
            continue;
        }
        const u64 KeyFrameID = KeyFrameIDs[j];
        typeKeyFrame& KeyFrame = GlobalMap->KeyFrames[KeyFrameID];
        const u64 ImagePointID = ImagePointIDs[j];
        KeyFrame.Points.ImagePoints[ImagePointID].MapPointID = PANTO_ID_NOT_SET;
    }
    GlobalMap->MapPoints.remove(MapPointIndex);
}

void MAP_LogGlobalMap(const typeGlobalMap& GlobalMap)
{
    LG_Log( LogSeverity::DBG,
            "[MAP_LogGlobalMap] KeyFrames: active = %zu, size = %zu | MapPoints: active = %zu, size = %zu | Age = %llu\n",
            GlobalMap.KeyFrames.active_size(),
            GlobalMap.KeyFrames.size(),
            GlobalMap.MapPoints.active_size(),
            GlobalMap.MapPoints.size(),
            GlobalMap.Age);

    for(std::size_t i{}; i < GlobalMap.KeyFrames.size(); i++)
    {
        if(!GlobalMap.KeyFrames.contains(i))
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[MAP_LogGlobalMap] KF slot %zu = EMPTY\n",
                    i);

            continue;
        }

        const typeKeyFrame& KeyFrame =
            GlobalMap.KeyFrames[i];

        LG_Log(
                LogSeverity::DBG,
                "[MAP_LogGlobalMap] KF slot %zu: ID = %llu, ImagePoints = %zu/%zu, timestamp = %f\n",
                i,
                KeyFrame.ID,
                KeyFrame.Points.ImagePoints.active_size(),
                KeyFrame.Points.ImagePoints.size(),
                KeyFrame.Camera.TimeStamp);

        if(KeyFrame.ID != i)
        {
            LG_Log(
                    LogSeverity::ERROR,
                    "[MAP_LogGlobalMap] ERROR: KF slot %zu contains ID %llu\n",
                    i,
                    KeyFrame.ID);
        }
    }

    for(std::size_t i{}; i < GlobalMap.MapPoints.size(); i++)
    {
        if(!GlobalMap.MapPoints.contains(i))
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[MAP_LogGlobalMap] MP slot %zu = EMPTY\n",
                    i);

            continue;
        }

        const typePantoMapPoint& MapPoint =
            GlobalMap.MapPoints[i];

        LG_Log(
                LogSeverity::DBG,
                "[MAP_LogGlobalMap] MP slot %zu: ID = %llu, observations = %zu, visible = %llu, found = %llu\n",
                i,
                MapPoint.ID,
                MapPoint.KeyFrameIDs.active_size(),
                MapPoint.NumVisible,
                MapPoint.NumFound);

        if(MapPoint.ID != i)
        {
            LG_Log(
                    LogSeverity::ERROR,
                    "[MAP_LogGlobalMap] ERROR: MP slot %zu contains ID %llu\n",
                    i,
                    MapPoint.ID);
        }

        if(MapPoint.KeyFrameIDs.size() !=
           MapPoint.ImagePointIDs.size())
        {
            LG_Log(
                    LogSeverity::ERROR,
                    "[MAP_LogGlobalMap] ERROR: MP %llu observation backing sizes differ: KFIDs = %zu, ImagePointIDs = %zu\n",
                    MapPoint.ID,
                    MapPoint.KeyFrameIDs.size(),
                    MapPoint.ImagePointIDs.size());

            continue;
        }

        std::unordered_set<u64> ObservedKeyFrames;

        for(std::size_t j{}; j < MapPoint.KeyFrameIDs.size(); j++)
        {
            const bool HasKeyFrame =
                MapPoint.KeyFrameIDs.contains(j);

            const bool HasImagePoint =
                MapPoint.ImagePointIDs.contains(j);

            if(HasKeyFrame != HasImagePoint)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_LogGlobalMap] ERROR: MP %llu observation slot %zu occupancy mismatch: KF = %d, IP = %d\n",
                        MapPoint.ID,
                        j,
                        static_cast<i32>(HasKeyFrame),
                        static_cast<i32>(HasImagePoint));

                continue;
            }

            if(!HasKeyFrame)
            {
                continue;
            }

            const u64 KeyFrameID =
                MapPoint.KeyFrameIDs[j];

            const u64 ImagePointID =
                MapPoint.ImagePointIDs[j];

            LG_Log(
                    LogSeverity::DBG,
                    "[MAP_LogGlobalMap]   MP %llu observation %zu -> KF %llu, IP %llu\n",
                    MapPoint.ID,
                    j,
                    KeyFrameID,
                    ImagePointID);

            if(!ObservedKeyFrames.insert(KeyFrameID).second)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_LogGlobalMap] ERROR: MP %llu has duplicate observation in KF %llu\n",
                        MapPoint.ID,
                        KeyFrameID);
            }

            if(!GlobalMap.KeyFrames.contains(KeyFrameID))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_LogGlobalMap] ERROR: MP %llu references missing KF %llu\n",
                        MapPoint.ID,
                        KeyFrameID);

                continue;
            }

            const typeKeyFrame& KeyFrame =
                GlobalMap.KeyFrames[KeyFrameID];

            if(!KeyFrame.Points.ImagePoints.contains(ImagePointID))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_LogGlobalMap] ERROR: MP %llu references missing IP %llu in KF %llu\n",
                        MapPoint.ID,
                        ImagePointID,
                        KeyFrameID);

                continue;
            }

            const typePantoImagePoint& ImagePoint =
                KeyFrame.Points.ImagePoints[ImagePointID];

            if(ImagePoint.MapPointID != MapPoint.ID)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[MAP_LogGlobalMap] ERROR: MP %llu -> KF %llu IP %llu, but IP references MP %llu\n",
                        MapPoint.ID,
                        KeyFrameID,
                        ImagePointID,
                        ImagePoint.MapPointID);
            }
        }
    }
}

void MAP_LogGraphConsistency( const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph)
{
    const std::size_t MaxSize =
        std::max(GlobalMap.KeyFrames.size(),
                CovisibilityGraph.CovisibilityGraph.size());

    for(std::size_t i{}; i < MaxSize; i++)
    {
        const bool KeyFrameExists =
            i < GlobalMap.KeyFrames.size() &&
            GlobalMap.KeyFrames.contains(i);

        const bool GraphVertexExists =
            i < CovisibilityGraph.CovisibilityGraph.size() &&
            CovisibilityGraph.CovisibilityGraph.contains(i);

        if(KeyFrameExists != GraphVertexExists)
        {
            LG_Log(LogSeverity::ERROR,
                    "[MAP_LogGraphConsistency] ERROR: slot %zu KF = %d, Graph = %d\n",
                    i,
                    static_cast<i32>(KeyFrameExists),
                    static_cast<i32>(GraphVertexExists));
        }
    }
}

void MAP_LogMappingData(void)
{
    static bool IsLogged = false;

    if(IsLogged)
    {
        LG_Log(
            LogSeverity::DBG,
            "[MAP_LogMappingData] WARNING: Mapping data logged more than once"
        );
    }

    const u64 NumHighPixelErrors =
        MappingData.NumObservationEdgesPixelErrorHigh;

    double MeanPixelError = 0.0;
    double PixelErrorVariance = 0.0;
    double PixelErrorStandardDeviation = 0.0;

    if(NumHighPixelErrors > 0)
    {
        const double N = static_cast<double>(NumHighPixelErrors);

        MeanPixelError = MappingData.SumPixelErrorRemovedPixels / N;

        if(NumHighPixelErrors > 1)
        {
            PixelErrorVariance =
                (
                    MappingData.SquaredSumPixelErrorRemovedPixels -
                    (
                        MappingData.SumPixelErrorRemovedPixels *
                        MappingData.SumPixelErrorRemovedPixels
                    ) / N
                ) / (N - 1.0);

            // Protect against a tiny negative value caused by rounding.
            PixelErrorVariance = std::max(0.0, PixelErrorVariance);
            PixelErrorStandardDeviation = std::sqrt(PixelErrorVariance);
        }
    }

    LG_Log(
        LogSeverity::DATA,
        "\n"
        "======================= MAPPING DATA =======================\n"
        " Culling\n"
        "   Recent map points culled          : %llu\n"
        "   Keyframes culled                  : %llu\n"
        "   Map points culled                 : %llu\n"
        "\n"
        " Map-point fusion\n"
        "   New observations found            : %llu\n"
        "   Map-point fusions                 : %llu\n"
        "\n"
        " Observation edges\n"
        "   Total edges culled                : %llu\n"
        "   Pixel error too high              : %llu\n"
        "   Failed projection                 : %llu\n"
        "\n"
        " High pixel-error statistics\n"
        "   Mean pixel error                  : %.6f px\n"
        "   Pixel-error variance              : %.6f px^2\n"
        "   Pixel-error standard deviation    : %.6f px\n"
        "============================================================\n",
        static_cast<unsigned long long>(
            MappingData.RecentMapPointsCulled
        ),
        static_cast<unsigned long long>(
            MappingData.KeyFramesCulled
        ),
        static_cast<unsigned long long>(
            MappingData.MapPointsCulled
        ),
        static_cast<unsigned long long>(
            MappingData.MapPointFusionObservations
        ),
        static_cast<unsigned long long>(
            MappingData.MapPointFusions
        ),
        static_cast<unsigned long long>(
            MappingData.ObservationEdgesCulled
        ),
        static_cast<unsigned long long>(
            NumHighPixelErrors
        ),
        static_cast<unsigned long long>(
            MappingData.NumObservationEdgesFailedProjection
        ),
        MeanPixelError,
        PixelErrorVariance,
        PixelErrorStandardDeviation
    );

    IsLogged = true;
}

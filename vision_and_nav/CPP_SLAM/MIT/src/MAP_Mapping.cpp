#include "MAP_Mapping.hpp"
#include "MAPPriv_Mapping.hpp"
#include <unordered_map>

typeMappingData MappingData = 
{
    .RecentMapPointsCulled = 0,
    .KeyFramesCulled = 0,
    .MapPointsCulled = 0,

    .ObservationEdgesCulled = 0,
    .NumObservationEdgesPixelErrorHigh = 0,
    .NumObservationEdgesFailedProjection = 0,
    .SumPixelErrorRemovedPixels = 0.0,
    .SquaredSumPixelErrorRemovedPixels = 0.0
};

typeGlobalMap MAP_InitializeFromGT(const typeNavigationState& FirstNavState, const typeNavigationState& SecondNavState,
        const typePantoFrame& FirstFrame, const typePantoFrame& SecondFrame)
{
    typeKeyFrame FirstKF = KEY_CreateKeyFrame(FirstNavState, FirstFrame, 0);
    typeKeyFrame SecondKF = KEY_CreateKeyFrame(SecondNavState, SecondFrame, 1);

    typeGlobalMap GlobalMap
    {
        .KeyFrames{},
        .MapPoints{},
        .Age = 0
    }; 

    const std::vector<u64> Indexes = KEY_InsertNewMapPoints(FirstKF, SecondKF, GlobalMap.MapPoints, GlobalMap.Age);

    (void) MAP_AppendKeyFrame(GlobalMap, FirstKF);
    (void) MAP_AppendKeyFrame(GlobalMap, SecondKF);

    return GlobalMap;
}

u64 MAP_AppendKeyFrame(typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame)
{
    const u64 ID = GlobalMap.KeyFrames.push_back(KeyFrame);
    GlobalMap.KeyFrames[ID].ID = ID;
    GlobalMap.Age++;
    return ID;
}

typeLocalMapTracking MAP_CreateLocalMapTracking(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame)
{
    const std::size_t NumberOfKeyFrames = GlobalMap.KeyFrames.size();
    LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMapTracking] Number of KeyFrames in local map creation %zu\n", NumberOfKeyFrames);
    std::vector<u64> KeyFrameCount(NumberOfKeyFrames);
    typeLocalMapTracking LocalMap{};

    for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;

        if(MapPointID != PANTO_ID_NOT_SET)
        {
            assert(GlobalMap.MapPoints.contains(MapPointID));
            const typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];
            for(const u64& KeyFrameID : MapPoint.KeyFrameIDs)
            {
                KeyFrameCount[KeyFrameID]++;
            }
        }
    }

    std::unordered_set<u64> AddedKeyFrames;

    for(std::size_t i{}; i < NumberOfKeyFrames; i++)
    {
        if(!GlobalMap.KeyFrames.contains(i))
        {
            continue;
        }

        if(KeyFrameCount[i] > 0)
        {
            LocalMap.KeyFrameIDs.push_back(i);
            AddedKeyFrames.insert(i);
        }
    }

    std::unordered_set<u64> AddedMapPoints;

    for(const u64& KeyFrameID : LocalMap.KeyFrameIDs)
    {
        LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMapTracking] KeyFrame considered for mappoints %llu\n", KeyFrameID);
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                if(AddedMapPoints.insert(ImagePoint.MapPointID).second)
                {
                    LocalMap.MapPointIDs.push_back(ImagePoint.MapPointID);
                }
            }
        }
    }
    
    std::vector<u64> AdditionalKeyFrames;

    for(const u64& KeyFrameID : LocalMap.KeyFrameIDs)
    {
        if(LocalMap.KeyFrameIDs.size() + AdditionalKeyFrames.size() >= PANTO_MAX_LOCAL_TRACKING_MAP_SIZE)
        {
            break;
        }

        typeCovisibility MostCovisibleFrame = GRAPH_GetMostCovisibleFrame(CovisibilityGraph, KeyFrameID);
        const u64 MostCovisibleFrameID = MostCovisibleFrame.KeyFrameID;
        LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMapTracking] Most covisible KeyFrameID in Local Map creation %llu\n", MostCovisibleFrameID);

        if(MostCovisibleFrameID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        if(!GlobalMap.KeyFrames.contains(MostCovisibleFrameID))
        {
            continue;
        }

        if(AddedKeyFrames.insert(MostCovisibleFrameID).second)
        {
            AdditionalKeyFrames.push_back(MostCovisibleFrameID);
            for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[MostCovisibleFrameID].Points.ImagePoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET && AddedMapPoints.insert(MapPointID).second)
                {
                    LocalMap.MapPointIDs.push_back(MapPointID);
                }
            }
        }
    }

    LocalMap.KeyFrameIDs.insert(LocalMap.KeyFrameIDs.end(), AdditionalKeyFrames.begin(), AdditionalKeyFrames.end());

    return LocalMap;
}

typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const u64 LatestKeyFrameID)
{
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);
    typeLocalMap LocalMap;
    std::unordered_set<u64> KeyFrameInLocalMap;

    LocalMap.KeyFrameIDs.push_back(LatestKeyFrameID);
    KeyFrameInLocalMap.insert(LatestKeyFrameID);

    for(const typeCovisibility& Covisibility : MostCovisible)
    {
        LocalMap.KeyFrameIDs.push_back(Covisibility.KeyFrameID);
        KeyFrameInLocalMap.insert(Covisibility.KeyFrameID);
    }

    std::unordered_set<u64> MapPointInLocalMap;

    for(const u64 KeyFrameID : LocalMap.KeyFrameIDs)
    {
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }
            if(MapPointInLocalMap.insert(ImagePoint.MapPointID).second)
            {
                LocalMap.MapPointIDs.push_back(ImagePoint.MapPointID);
            }
        }
    }

    for(const u64 LocalMapPointID : LocalMap.MapPointIDs)
    {
        typePantoVector<u64> KeyFrameIDs = GlobalMap.MapPoints[LocalMapPointID].KeyFrameIDs;

        for(const u64 KeyFrameID : KeyFrameIDs)
        {
            if(KeyFrameInLocalMap.insert(KeyFrameID).second)
            {
                LocalMap.FixedKeyFrameIDs.push_back(KeyFrameID);
            }
        }
    }

    return LocalMap;
}

typePantoVector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame)
{
    const typePantoKeypointFrame& LastKeyFramePoints = LastKeyFrame.Points;
    typePantoVector<typePantoMapPoint> LastKeyFrameMapPoints;
    const typePantoVector<typePantoMapPoint>& MapPoints = Map.MapPoints;

    for(const typePantoImagePoint& ImagePoint : LastKeyFramePoints.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
        {
            LastKeyFrameMapPoints.push_back(MapPoints[ImagePoint.MapPointID]);
        }
    }
    return LastKeyFrameMapPoints;
}

typeLocalMapInfo MAP_MatchMapPointLocalMap(typeGlobalMap& GlobalMap, typeLocalMapTracking& LocalMap, typeKeyFrame& NewKeyFrame)
{
    std::vector<typePantoMapPoint> LocalMapPoints;
    LocalMapPoints.reserve(LocalMap.MapPointIDs.size());
    for(const u64 LocalMapPointID : LocalMap.MapPointIDs)
    {
        typePantoMapPoint& MapPoint = GlobalMap.MapPoints[LocalMapPointID];
        LocalMapPoints.push_back(MapPoint);
    }

    LG_Log(LogSeverity::DBG, "[MAP_MatchMapPointLocalMap] Getting median scene depth");
    const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(NewKeyFrame, LocalMapPoints);

    const typeCamera& Camera = NewKeyFrame.Camera;
    LG_Log(LogSeverity::DBG, "[MAP_MatchMapPointLocalMap] Matching mappoints to keyframe");
    u64 NumProjectedMapPoints = 0;
    const u64 NumTrackedMapPoints = PT_MatchMapPointsToKeyFrame(
            NewKeyFrame.Points,
            LocalMapPoints,
            Camera,
            GlobalMap.MapPoints,
            &NumProjectedMapPoints);

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
        .MedianDepth = MedianDepth
    };

    return LocalMapInfo;
}

void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeCovisibilityGraph& CovisibilityGraph, const u64 CurrentFrameID)
{
    assert(GlobalMap.KeyFrames.contains(CurrentFrameID));
    assert(CovisibilityGraph.contains(CurrentFrameID));

    std::vector<u64> CovisibleKeyFrameIDs;
    CovisibleKeyFrameIDs.reserve(CovisibilityGraph[CurrentFrameID].size());

    for(const auto& [KeyFrameID, Weight] : CovisibilityGraph[CurrentFrameID])
    {
        if(Weight == 0)
        {
            continue;
        }

        assert(GlobalMap.KeyFrames.contains(KeyFrameID));
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

        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameID];

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

            const typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];
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
        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[CulledID];
        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
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

            typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];
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
        GlobalMap.KeyFrames.remove(CulledID);
        LG_Log(LogSeverity::DBG, "[MAP_CullLocalMap] culled keyframe %llu\n", CulledID);
    }

    LG_Log(LogSeverity::DBG, "[MAP_CullLocalMap] Culled %zu kfs",
            CulledKeyFrameIDs.size());
}

void MAP_CullRecentMapPoints(typePantoVector<u64>& RecentMapPointIndexes, typeGlobalMap& GlobalMap)
{
    LG_Log( LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culling recent mappoints\n");

    std::vector<u64> RemoveRecentIndexes;

    u64 NumRemoved = 0;

    for(std::size_t i{}; i < RecentMapPointIndexes.size(); i++)
    {
        if(!RecentMapPointIndexes.contains(i))
        {
            continue;
        }

        const u64 MapPointID =
            RecentMapPointIndexes[i];

        if(!GlobalMap.MapPoints.contains(MapPointID))
        {
            RemoveRecentIndexes.push_back(i);
            continue;
        }

        typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];
        const u64 MapPointIndex = MapPoint.ID;
        const u64 Age = GlobalMap.Age - MapPoint.CreationAge;
        const u64 NumObservations = static_cast<u64>(MapPoint.KeyFrameIDs.active_size());
        if(PT_GetFoundRatio(MapPoint) < PANTO_MIN_FOUND_RATIO)
        {
            RecentMapPointIndexes.remove(i);
            MAPPriv_CullRecentMapPoint(MapPoint, MapPointIndex, GlobalMap);
            NumRemoved++;
            MappingData.RecentMapPointsCulled++;
            continue;
        }
        else if(Age >= 2 && NumObservations <= 2)
        {
            RecentMapPointIndexes.remove(i);
            MAPPriv_CullRecentMapPoint(MapPoint, MapPointIndex, GlobalMap);
            NumRemoved++;
            MappingData.RecentMapPointsCulled++;
            continue;
        }
        else if(Age >= 3)
        {
            RecentMapPointIndexes.remove(i);
        }
    }

    for(const u64 Index : RemoveRecentIndexes)
    {
        RecentMapPointIndexes.remove(Index);
    }

    LG_Log(LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culled %llu mappoints\n", NumRemoved);
}

void MAP_CullObservationEdges( typeGlobalMap& GlobalMap, typeCovisibilityGraph& CovisibilityGraph)
{
    const std::size_t InitialMapPointCount = GlobalMap.MapPoints.active_size();

    u64 NumMapPointsWithBadEdges = 0;
    u64 NumCulledObservationEdges = 0;

    fp64 MaxError = 0.0;
    fp64 SumError = 0.0;
    u64 NumErrors = 0;
    u64 NumAboveThreshold = 0;

    std::vector<u64> CulledMapPointIDs;

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Starting with %zu active map points\n",
            InitialMapPointCount);

    for(typePantoMapPoint& MapPoint : GlobalMap.MapPoints)
    {
        typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;

        typePantoVector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;

        assert( KeyFrameIDs.size() == ImagePointIDs.size());

        const u64 NumObservationsBefore = PT_GetNumObservations(MapPoint);

        Eigen::Vector2d ProjectedPoint;

        std::vector<u64> CulledIndexes;

        for(std::size_t i{}; i < KeyFrameIDs.size(); i++)
        {
            assert( KeyFrameIDs.contains(i) == ImagePointIDs.contains(i));

            if(!KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = KeyFrameIDs[i];

            const u64 ImagePointID = ImagePointIDs[i];

            assert(GlobalMap.KeyFrames.contains( KeyFrameID));

            typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameID];

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

        if(RemainingObservations < 1)
        {
            LG_Log( LogSeverity::DBG, "[MAP_CullObservationEdges] MP %llu marked for full deletion: remaining observations = %llu\n",
                    MapPoint.ID, RemainingObservations);
            CulledMapPointIDs.push_back( MapPoint.ID);
            MappingData.MapPointsCulled++;

            continue;
        }

        for(const u64 CulledIndex : CulledIndexes)
        {
            assert( KeyFrameIDs.contains( CulledIndex));
            assert( ImagePointIDs.contains( CulledIndex));

            const u64 KeyFrameID = KeyFrameIDs[CulledIndex];
            const u64 ImagePointID = ImagePointIDs[CulledIndex];

            typePantoImagePoint& ImagePoint = GlobalMap.KeyFrames[ KeyFrameID]. Points.ImagePoints[ ImagePointID];
            GRAPH_DecrementAllOther( CovisibilityGraph, KeyFrameIDs, CulledIndex);

            ImagePoint.MapPointID = PANTO_ID_NOT_SET;
            KeyFrameIDs.remove( CulledIndex);
            ImagePointIDs.remove( CulledIndex);
            NumCulledObservationEdges++;
        }
    }

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] MapPoints with bad edges = %llu\n",
            NumMapPointsWithBadEdges);

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Removing %zu entire map points\n",
            CulledMapPointIDs.size());

    for(const u64 CulledMapPointID : CulledMapPointIDs)
    {
        assert( GlobalMap.MapPoints.contains( CulledMapPointID));

        typePantoMapPoint& MapPoint = GlobalMap.MapPoints[ CulledMapPointID];

        typePantoVector<u64>& KeyFrameIDs = MapPoint.KeyFrameIDs;

        typePantoVector<u64>& ImagePointIDs = MapPoint.ImagePointIDs;

        assert( KeyFrameIDs.size() == ImagePointIDs.size());

        LG_Log( LogSeverity::DBG,
                "[MAP_CullObservationEdges] Fully removing MP %llu with %llu observations\n",
                CulledMapPointID,
                PT_GetNumObservations(MapPoint));

        GRAPH_DecrementAll( CovisibilityGraph, KeyFrameIDs);

        for(std::size_t i{}; i < KeyFrameIDs.size(); i++)
        {
            assert( KeyFrameIDs.contains(i) == ImagePointIDs.contains(i));

            if(!KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID = KeyFrameIDs[i];

            const u64 ImagePointID = ImagePointIDs[i];

            assert( GlobalMap.KeyFrames.contains(
                            KeyFrameID));

            assert(GlobalMap.KeyFrames[ KeyFrameID]. Points.ImagePoints.contains(
                            ImagePointID));

            typePantoImagePoint& ImagePoint = GlobalMap.KeyFrames[ KeyFrameID].
                    Points.ImagePoints[
                        ImagePointID];

            ImagePoint.MapPointID = PANTO_ID_NOT_SET;
        }

        GlobalMap.MapPoints.remove( CulledMapPointID);
    }

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] Removed %llu observation edges\n",
            NumCulledObservationEdges);

    LG_Log( LogSeverity::DBG,
            "[MAP_CullObservationEdges] MapPoints: %zu -> %zu\n",
            InitialMapPointCount,
            GlobalMap.MapPoints.active_size());

    LG_Log(LogSeverity::DBG, 
            "[MAP_CullObservationEdges] Mean pixel error: %lf, Max: %lf, NumAbove: %llu\n",
            SumError / static_cast<fp64>(NumErrors),
            MaxError,
            NumAboveThreshold
          );
}

std::vector<u64> MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeKeyFrame& NewKeyFrame, const typeCovisibilityGraph& CovisibilityGraph,
        const u64 LatestKeyFrameID)
{
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);
    std::vector<typeKeyFrame> LocalMapKeyFrames;

    for(const typeCovisibility& Covisibility : MostCovisible)
    {
        LocalMapKeyFrames.push_back(GlobalMap.KeyFrames[Covisibility.KeyFrameID]);
    }

    std::unordered_set<u64> LocalMapPointIDs;

    for(const typeKeyFrame& KeyFrame : LocalMapKeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint :
                GlobalMap.KeyFrames[KeyFrame.ID].Points.ImagePoints)
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
        if(GlobalMap.MapPoints.contains(MapPointID))
        {
            LocalMapMapPoints.push_back(GlobalMap.MapPoints[MapPointID]);
        }
    }

    Eigen::Vector3d NewCameraCenter = CM_GetCameraCenter(NewKeyFrame.Camera);
    std::vector<u64> NewPointIndexes;
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

        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameLocal.ID];

        if(PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth))
        {
            const std::vector<u64> Index = KEY_InsertNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap.MapPoints, GlobalMap.Age);
            NewPointIndexes.insert(NewPointIndexes.end(), Index.begin(), Index.end());
        }
    }

    return NewPointIndexes;
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
    assert(GlobalMap.KeyFrames.size() == CovisibilityGraph.size());

    for(std::size_t i{}; i < GlobalMap.KeyFrames.size(); i++)
    {
        assert(GlobalMap.KeyFrames.contains(i) == CovisibilityGraph.contains(i));

        if(GlobalMap.KeyFrames.contains(i))
        {
            assert(GlobalMap.KeyFrames[i].ID == i);
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

void MAPPriv_CullRecentMapPoint(typePantoMapPoint& MapPoint, u64 MapPointIndex, typeGlobalMap& GlobalMap)
{
    typePantoVector KeyFrameIDs = MapPoint.KeyFrameIDs;
    typePantoVector ImagePointIDs = MapPoint.ImagePointIDs;
    for(std::size_t j{}; j < KeyFrameIDs.size(); j++)
    {
        if(!KeyFrameIDs.contains(j) || !ImagePointIDs.contains(j)) 
        {
            continue;
        }
        const u64 KeyFrameID = KeyFrameIDs[j];
        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameID];
        const u64 ImagePointID = ImagePointIDs[j];
        KeyFrame.Points.ImagePoints[ImagePointID].MapPointID = PANTO_ID_NOT_SET;
    }
    GlobalMap.MapPoints.remove(MapPointIndex);
}

void MAP_LogGlobalMap(const typeGlobalMap& GlobalMap)
{
    LG_Log(
            LogSeverity::DBG,
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
        std::max(
                GlobalMap.KeyFrames.size(),
                CovisibilityGraph.size());

    for(std::size_t i{}; i < MaxSize; i++)
    {
        const bool KeyFrameExists =
            i < GlobalMap.KeyFrames.size() &&
            GlobalMap.KeyFrames.contains(i);

        const bool GraphVertexExists =
            i < CovisibilityGraph.size() &&
            CovisibilityGraph.contains(i);

        if(KeyFrameExists != GraphVertexExists)
        {
            LG_Log(
                    LogSeverity::ERROR,
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

        MeanPixelError =
            MappingData.SumPixelErrorRemovedPixels / N;

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

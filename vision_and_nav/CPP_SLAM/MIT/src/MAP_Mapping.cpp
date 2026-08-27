#include "MAP_Mapping.hpp"
#include "MAPPriv_Mapping.hpp"
#include <unordered_map>

void MAP_AppendKeyFrame(typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame)
{
    GlobalMap.KeyFrames.push_back(KeyFrame);
}

void MAP_InsertPreliminaryKeyFrame(typeGlobalMap& Map, typeKeyFrame& KeyFrame)
{
    const u64 ID = Map.KeyFrames.size();
    KeyFrame.ID = ID;
    Map.KeyFrames.push_back(KeyFrame);
}

void MAP_RemovePreliminaryKeyFrame(typeGlobalMap& Map)
{
    KEY_NonValidKeyFrame();
    Map.KeyFrames.pop_back();
}

typeLocalMapTracking MAP_CreateLocalMapTracking(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame)
{
    const std::size_t NumberOfKeyFrames = GlobalMap.KeyFrames.size();
    LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMap] Number of KeyFrames in local map creation %zu\n", NumberOfKeyFrames);
    std::vector<u64> KeyFrameCount(NumberOfKeyFrames);
    typeLocalMapTracking LocalMap{};

    for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID != PANTO_ID_NOT_SET)
        {
            const typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointID];
            for(const u64& KeyFrameID : MapPoint.KeyFrameIDs)
            {
                LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMap] KeyFrameID in Local Map creation %llu\n", KeyFrameID);
                KeyFrameCount[KeyFrameID]++;
            }
        }
    }

    std::unordered_set<u64> AddedKeyFrames;

    for(std::size_t i{}; i < NumberOfKeyFrames; i++)
    {
        if(KeyFrameCount[i] > 0)
        {
            LocalMap.KeyFrameIDs.push_back(GlobalMap.KeyFrames[i].ID);
            AddedKeyFrames.insert(GlobalMap.KeyFrames[i].ID);
        }
    }

    std::unordered_set<u64> AddedMapPoints;

    for(const u64& KeyFrameID : LocalMap.KeyFrameIDs)
    {
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrameID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                if(AddedMapPoints.insert(ImagePoint.MapPointID).second)
                {
                    LocalMap.MapPointIDs.push_back(
                            GlobalMap.MapPoints[ImagePoint.MapPointID].ID);
                }
            }
        }
    }

    for(const u64& KeyFrameID : LocalMap.KeyFrameIDs)
    {
        if(LocalMap.KeyFrameIDs.size() > PANTO_MAX_LOCAL_TRACKING_MAP_SIZE)
        {
            break;
        }

        typeCovisibility MostCovisibleFrame = GRAPH_GetMostCovisibleFrame(CovisibilityGraph, KeyFrameID);
        const u64 MostCovisibleFrameID = MostCovisibleFrame.KeyFrameID;
        if(AddedKeyFrames.insert(MostCovisibleFrameID).second)
        {
            LocalMap.KeyFrameIDs.push_back(MostCovisibleFrameID);
            for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[MostCovisibleFrameID].Points.ImagePoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET || AddedMapPoints.insert(MapPointID).second)
                {
                    LocalMap.MapPointIDs.push_back(MapPointID);
                }
            }
        }
    }

    return LocalMap;
}

typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph)
{
    const u64 LatestKeyFrameID = GlobalMap.KeyFrames.back().ID;
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);
    typeLocalMap LocalMap;
    std::unordered_set<u64> KeyFrameInLocalMap;
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
                LocalMap.MapPointIDs.push_back(GlobalMap.MapPoints[ImagePoint.MapPointID].ID);
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
                LocalMap.FixedKeyFrameIDs.push_back(GlobalMap.KeyFrames[KeyFrameID].ID);
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

    const u64 NumLocalMapPoints = static_cast<u64>(LocalMapPoints.size());
    const typeCamera& Pose = NewKeyFrame.Pose;
    LG_Log(LogSeverity::DBG, "[MAP_MatchMapPointLocalMap] Matching mappoints to keyframe");
    const u64 NumTrackedMapPoints = PT_MatchMapPointsToKeyFrame(NewKeyFrame.Points, LocalMapPoints, Pose);

    const fp64 TrackingRatio = static_cast<fp64>(NumTrackedMapPoints) / static_cast<fp64>(NumLocalMapPoints);

    typeLocalMapInfo LocalMapInfo = 
    {
        .TrackedRatio = TrackingRatio,
        .MedianDepth = MedianDepth
    };

    return LocalMapInfo;
}

void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap)
{
    return;
}

void MAP_CullRecentMapPoints(typePantoVector<u64>& RecentMapPointIndexes, typeGlobalMap& GlobalMap)
{
    LG_Log(LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culling recent mappoints\n");
    u64 NumRemoved = 0;
    const u64 CurrentKeyFrameID = GlobalMap.KeyFrames.back().ID;
    for(std::size_t i{}; i < RecentMapPointIndexes.size(); i++)
    {
        if(!RecentMapPointIndexes.contains(i))
        {
            continue;
        }
        const u64 MapPointIndex = RecentMapPointIndexes[i];
        typePantoMapPoint& MapPoint = GlobalMap.MapPoints[MapPointIndex];
        const u64 Age = CurrentKeyFrameID - MapPoint.FirstKFKID;
        const u64 NumObservations = static_cast<u64>(MapPoint.KeyFrameIDs.active_size());
        if(PT_GetFoundRatio(MapPoint) < PANTO_MIN_FOUND_RATIO)
        {
            RecentMapPointIndexes.remove(i);
            MAPPriv_CullRecentMapPoint(MapPoint, MapPointIndex, GlobalMap);
            NumRemoved++;
            continue;
        }
        else if(Age >= 2 && NumObservations <= 2)
        {
            RecentMapPointIndexes.remove(i);
            MAPPriv_CullRecentMapPoint(MapPoint, MapPointIndex, GlobalMap);
            NumRemoved++;
            continue;
        }
        else if(Age >= 3)
        {
            RecentMapPointIndexes.remove(i);
        }
    }
    LG_Log(LogSeverity::DBG, "[MAP_CullRecentMapPoints] Culled %llu mappoints\n", NumRemoved);
}

std::vector<u64> MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeKeyFrame& NewKeyFrame, const typeCovisibilityGraph& CovisibilityGraph)
{
    const u64 LatestKeyFrameID = GlobalMap.KeyFrames.back().ID;
    std::vector<typeCovisibility> MostCovisible = GRAPH_GetTopNCovisibleFrames(CovisibilityGraph, LatestKeyFrameID, PANTO_TOP_N_KF_FOR_LOCAL_MAP);

    std::vector<typeKeyFrame> LocalMapKeyFrames;
    std::vector<typePantoMapPoint> LocalMapMapPoints;
    for(const typeCovisibility& Covisibility : MostCovisible)
    {
        LocalMapKeyFrames.push_back(GlobalMap.KeyFrames[Covisibility.KeyFrameID]);
    }
    for(const typeKeyFrame& KeyFrame : LocalMapKeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint : GlobalMap.KeyFrames[KeyFrame.ID].Points.ImagePoints)
        {
            if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
            {
                continue;
            }
            LocalMapMapPoints.push_back(GlobalMap.MapPoints[ImagePoint.MapPointID]);
        }
    }

    Eigen::Vector3d NewCameraCenter = CM_GetCameraCenter(NewKeyFrame.Pose);

    std::vector<u64> NewPointIndexes;

    for(const typeKeyFrame& KeyFrameLocal : LocalMapKeyFrames)
    {
        // Ignore new keyframe
        if(KeyFrameLocal.ID == NewKeyFrame.ID)
        {
            continue;
        }

        const Eigen::Vector3d CameraCenter = CM_GetCameraCenter(KeyFrameLocal.Pose);

        const fp64 BaseLine = (NewCameraCenter - CameraCenter).norm();

        const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(KeyFrameLocal, LocalMapMapPoints);

        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Median Depth in local map = %lf\n", MedianDepth); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline in between keyframes = %lf\n", BaseLine); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline is large enough = %d \n", PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth)); 

        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameLocal.ID];

        if(PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth))
        {
            const std::vector<u64> Index = KEY_InsertNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap.MapPoints);
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
        const typeCamera& Pose = KeyFrame.Pose;

        LG_Log(
            LogSeverity::DBG,
            "[MAP_LogGlobalMapPoses] KeyFrame %llu q = (%f, %f, %f, %f), t = (%f, %f, %f)\n",
            static_cast<unsigned long long>(KeyFrame.ID),
            Pose.Parameters.q.w(),
            Pose.Parameters.q.x(),
            Pose.Parameters.q.y(),
            Pose.Parameters.q.z(),
            Pose.Parameters.t[0],
            Pose.Parameters.t[1],
            Pose.Parameters.t[2]);
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

        const typePantoMapPoint& MapPoint =
            GlobalMapPoints[ImagePoint.MapPointID];

        Eigen::Vector2d ProjectedPoint{};

        if(!PROJ_Project(MapPoint.Point, ProjectedPoint, KeyFrame.Pose))
        {
            NumFailedProjection++;
            continue;
        }

        const fp64 Error =
            (ImagePoint.Point - ProjectedPoint).norm();

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
                KeyFrame.Pose.Pose.R;

            Transform.block<3, 1>(0, 3) =
                KeyFrame.Pose.Pose.t;

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
    assert(GlobalMap.KeyFrames[0].Pose.Intrinsics != nullptr);

    const Eigen::Matrix3d K =
        GlobalMap.KeyFrames[0].Pose.Intrinsics->K;

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

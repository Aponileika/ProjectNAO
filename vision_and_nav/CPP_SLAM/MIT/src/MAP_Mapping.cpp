#include "MAP_Mapping.hpp"
#include <cstddef>

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

typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame)
{
    const std::size_t NumberOfKeyFrames = GlobalMap.KeyFrames.size();
    LG_Log(LogSeverity::DBG, "[MAP_CreateLocalMap] Number of KeyFrames in local map creation %zu\n", NumberOfKeyFrames);
    std::vector<u64> KeyFrameCount(NumberOfKeyFrames);
    typeLocalMap LocalMap{};

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

    for(std::size_t i{}; i < NumberOfKeyFrames; i++)
    {
        if(KeyFrameCount[i] > 0)
        {
            LocalMap.KeyFrames.push_back(GlobalMap.KeyFrames[i]);
        }
    }

    std::unordered_set<u64> AddedMapPoints;

    for(const typeKeyFrame& KeyFrame : LocalMap.KeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                if(AddedMapPoints.insert(ImagePoint.MapPointID).second)
                {
                    LocalMap.MapPoints.push_back(
                            GlobalMap.MapPoints[ImagePoint.MapPointID]);
                }
            }
        }
    }

    return LocalMap;
}

std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame)
{
    const typePantoKeypointFrame& LastKeyFramePoints = LastKeyFrame.Points;
    std::vector<typePantoMapPoint> LastKeyFrameMapPoints;
    const std::vector<typePantoMapPoint>& MapPoints = Map.MapPoints;

    for(const typePantoImagePoint& ImagePoint : LastKeyFramePoints.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
        {
            LastKeyFrameMapPoints.push_back(MapPoints[ImagePoint.MapPointID]);
        }
    }
    return LastKeyFrameMapPoints;
}

typeLocalMapInfo MAP_MatchMapPointLocalMap(const typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame)
{
    const std::vector<typePantoMapPoint>& LocalMapPoints = LocalMap.MapPoints;

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

u64 MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame)
{
    const std::size_t SizeBefore = GlobalMap.MapPoints.size();
    const Eigen::Vector3d NewCameraCenter = CM_GetCameraCenter(NewKeyFrame.Pose);

    for(typeKeyFrame& KeyFrameLocal : LocalMap.KeyFrames)
    {
        // Ignore new keyframe
        if(KeyFrameLocal.ID == NewKeyFrame.ID)
        {
            continue;
        }

        const Eigen::Vector3d CameraCenter = CM_GetCameraCenter(KeyFrameLocal.Pose);

        const fp64 BaseLine = (NewCameraCenter - CameraCenter).norm();

        const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(KeyFrameLocal, LocalMap.MapPoints);

        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Median Depth in local map = %lf\n", MedianDepth); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline in between keyframes = %lf\n", BaseLine); 
        LG_Log(LogSeverity::DBG, "[MAP_CreateNewMapPoints] Baseline is large enough = %d \n", PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth)); 

        typeKeyFrame& KeyFrame = GlobalMap.KeyFrames[KeyFrameLocal.ID];

        if(PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth))
        {
            KEY_InsertNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap.MapPoints);
        }
    }
    const std::size_t NumNewPoints = GlobalMap.MapPoints.size() - SizeBefore;
    return static_cast<u64>(NumNewPoints);
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


void MAP_LogKeyFrameProjectionError(const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& GlobalMapPoints)
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



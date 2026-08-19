#include "MAP_Mapping.hpp"
#include <cstddef>

void MAP_InitializeGlobalMap(, const std::vector<typeKeyFrame>& KeyFrames)
{
    Map.KeyFrames = KeyFrames;
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
                KeyFrameCount[KeyFrameID]++;
            }
        }
    }

    for(const typeKeyFrame& KeyFrame : LocalMap.KeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                LocalMap.MapPoints.push_back(GlobalMap.MapPoints[ImagePoint.MapPointID]);
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

    return LocalMap;
}

std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& NewKeyFrame)
{
    const typePantoKeypointFrame& LastKeyFrame = NewKeyFrame.Points;
    std::vector<typePantoMapPoint> LastKeyFrameMapPoints;
    const std::vector<typePantoMapPoint>& MapPoints = Map.MapPoints;

    for(const typePantoImagePoint& ImagePoint : LastKeyFrame.ImagePoints)
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

    const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(NewKeyFrame, LocalMapPoints);

    const u64 NumLocalMapPoints = static_cast<u64>(LocalMapPoints.size());
    const typeCamera& Pose = NewKeyFrame.Pose;
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

u64 MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap, typeKeyFrame NewKeyFrame)
{
    const std::size_t SizeBefore = GlobalMap.MapPoints.size();
    const Eigen::Vector3d NewCameraCenter = CM_GetCameraCenter(NewKeyFrame.Pose);

    for(typeKeyFrame& KeyFrame : LocalMap.KeyFrames)
    {
        // Ignore new keyframe
        if(KeyFrame.ID == NewKeyFrame.ID)
        {
            continue;
        }

        const Eigen::Vector3d CameraCenter = CM_GetCameraCenter(KeyFrame.Pose);

        const fp64 BaseLine = (NewCameraCenter - CameraCenter).norm();

        const fp64 MedianDepth = KEY_GetLocalMapMedianDepth(KeyFrame, LocalMap.MapPoints);

        if(PANTO_BASELINE_LARGE_ENOUGH_TRIANGULATION(BaseLine, MedianDepth))
        {
            KEY_InsertNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap.MapPoints);
        }
    }
    const std::size_t NumNewPoints = GlobalMap.MapPoints.size() - SizeBefore;
    return static_cast<u64>(NumNewPoints);
}


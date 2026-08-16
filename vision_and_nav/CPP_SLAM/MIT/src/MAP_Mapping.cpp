#include "MAP_Mapping.hpp"
#include <cstddef>

void MAP_InsertPreliminaryKeyFrame(typeGlobalMap& Map, typeKeyFrame& KeyFrame)
{
    const u64 ID = Map.KeyFrames.size();
    KeyFrame.ID = ID;
    Map.KeyFrames.push_back(KeyFrame);
}

void MAP_RemovePreliminaryKeyFrame(typeGlobalMap& Map)
{
    KEY_PopKeyFrame();
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

fp64 MAP_MatchMapPointLocalMap(const typeGlobalMap& GlobalMap, const typeLocalMap& LocalMap, typeKeyFrame& NewKeyFrame)
{
    std::vector<typePantoMapPoint> MapPoints;
    for(const typeKeyFrame& KeyFrame : LocalMap.KeyFrames)
    {
        for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                MapPoints.push_back(GlobalMap.MapPoints[ImagePoint.MapPointID]);
            }
        }
    }
    const u64 NumLocalMapPoints = static_cast<u64>(MapPoints.size());
    const typeCamera& Pose = NewKeyFrame.Pose;
    const u64 NumTrackedMapPoints = PT_MatchMapPointsToKeyFrame(NewKeyFrame.Points, MapPoints, Pose);
    fp64 TrackingRatio = static_cast<fp64>(NumTrackedMapPoints) / static_cast<fp64>(NumLocalMapPoints);
    return TrackingRatio;
}

void MAP_CullLocalMap(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap)
{
    return;
}

u64 MAP_CreateNewMapPoints(typeGlobalMap& GlobalMap, typeLocalMap& LocalMap, typeKeyFrame NewKeyFrame)
{
    const std::size_t SizeBefore = GlobalMap.MapPoints.size();
    for(typeKeyFrame& KeyFrame : LocalMap.KeyFrames)
    {
        // Ignore new keyframe
        if(KeyFrame.ID == NewKeyFrame.ID)
        {
            continue;
        }

        KEY_GetNewMapPoints(NewKeyFrame, KeyFrame, GlobalMap.MapPoints);
    }
    const std::size_t NumNewPoints = GlobalMap.MapPoints.size() - SizeBefore;
    return static_cast<u64>(NumNewPoints);
}


#include "MAP_Mapping.hpp"
#include <cstddef>

void MAP_InsertKeyFrame(typeGlobalMap& Map, const typeKeyFrame& KeyFrame)
{
    Map.KeyFrames.push_back(KeyFrame);
}

typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeKeyFrame& KeyFrame)
{
    const std::size_t NumberOfKeyFrames = GlobalMap.KeyFrames.size();
    std::vector<u64> KeyFrameCount(NumberOfKeyFrames);
    typeLocalMap LocalMap{};

    for(const std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
    {
        for(const typePantoImagePoint& ImagePoint : CellPoints)
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

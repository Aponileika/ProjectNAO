#include "../include/GRAPH_PantoGraph.hpp"
#include <cstring>

void GRAPH_AddKeyFrame(typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& GlobalMapPoints)
{
    const typePantoKeypointFrame& KeyPointFrame = KeyFrame.Points;
    std::vector<u64> CovisibilityCount(KeyFrame.ID, 0);

    for(const typePantoImagePoint& ImagePoint : KeyPointFrame.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID != PANTO_ID_NOT_SET)
        {
            for(const u64& KeyFrameID : GlobalMapPoints[MapPointID].KeyFrameIDs)
            {
                CovisibilityCount[KeyFrameID]++;
            }
        }
    }

    std::vector<typeCovisibility> Covisibility;

    for(u64 KeyFrameID{}; KeyFrameID <= KeyFrame.ID; KeyFrameID++)
    {
        u64 Count = CovisibilityCount[KeyFrameID];

        if(Count == 0)
        {
            continue;
        }

        Covisibility.push_back(
        {
            .KeyFrameID = KeyFrameID,
            .Covisibility = Count
        });
    }

    std::sort(Covisibility.begin(), Covisibility.end(), 
            [](const typeCovisibility& A, const typeCovisibility& B)
            {
                return A.Covisibility > B.Covisibility;
            }
    );

    CovisibilityGraph.push_back(Covisibility);
}

std::vector<typeCovisibility> GRAPH_GetAllCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID)
{
    return CovisibilityGraph[KeyFrameID];
}

std::vector<typeCovisibility> GRAPH_GetTopNCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID, const u64 N)
{
    const std::vector<typeCovisibility>& Requested = CovisibilityGraph[KeyFrameID];
    std::vector<typeCovisibility> TopN(N);
    std::memcpy(TopN.data(), Requested.data(), N);
    return TopN;
}


void GRAPH_UpdateCovisibility(typeCovisibilityGraph& CovisibilityGraph, const typeLocalMap& LocalMap, const std::vector<typePantoMapPoint>& GlobalMapPoints)
{
    // TODO
}

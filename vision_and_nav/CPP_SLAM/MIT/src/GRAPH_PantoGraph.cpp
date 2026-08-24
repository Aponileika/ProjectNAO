#include "../include/GRAPH_PantoGraph.hpp"
#include <cstring>

void GRAPH_AddKeyFrame(typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& GlobalMapPoints,
        const u64 NumKeyFrames)
{
    const typePantoKeypointFrame& KeyPointFrame = KeyFrame.Points;
    std::vector<u64> CovisibilityCount(NumKeyFrames, 0);

    LG_Log(LogSeverity::DBG, "[GRAPH_AddKeyFrame] Adding KeyFrame %llu to CovisibilityGraph\n",KeyFrame.ID);

    for(const typePantoImagePoint& ImagePoint : KeyPointFrame.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID != PANTO_ID_NOT_SET)
        {
            for(const u64& KeyFrameID : GlobalMapPoints[MapPointID].KeyFrameIDs)
            {
                if(KeyFrameID == KeyFrame.ID)
                {
                    continue;
                }

                CovisibilityCount[KeyFrameID]++;
            }
        }
    }

    std::vector<typeCovisibility> Covisibility;

    for(u64 KeyFrameID{}; KeyFrameID < KeyFrame.ID; KeyFrameID++)
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


void GRAPH_UpdateCovisibility(typeCovisibilityGraph& CovisibilityGraph, const std::vector<typePantoMapPoint>& GlobalMapPoints, const u64 NumNewPoints,
        const u64 NewKeyFrameID)
{
    std::vector<u64> CovisibilityCount(CovisibilityGraph.size(), 0);
    LG_Log(LogSeverity::DBG, "[GRAPH_UpdateCovisibility] Covisibility grah size = %zu\n", CovisibilityGraph.size()); 
    
    for(auto GlobalMapIterator = GlobalMapPoints.end() - NumNewPoints; GlobalMapIterator != GlobalMapPoints.end();
            ++GlobalMapIterator)
    {
        for(const u64 KeyFrameID : GlobalMapIterator->KeyFrameIDs)
        {
            if(KeyFrameID == NewKeyFrameID)
            {
                continue;
            }
            ++CovisibilityCount[KeyFrameID];
        }
    }

    for(std::size_t i{}; i < CovisibilityGraph.size(); i++)
    {
        if(CovisibilityCount[i] == 0)
        {
            continue;
        }

        const u64 Count = CovisibilityCount[i];

        bool WasCovisible = false;
        for(typeCovisibility& Covisibility : CovisibilityGraph[NewKeyFrameID])
        {
            if(Covisibility.KeyFrameID == i)
            {
                Covisibility.Covisibility += Count;
                WasCovisible = true;
                break;
            }
        }

        if(!WasCovisible)
        {
            CovisibilityGraph[NewKeyFrameID].push_back(
                {
                    .KeyFrameID = static_cast<u64>(i),
                    .Covisibility = Count
                });
        }

        WasCovisible = false;
        for(typeCovisibility& Covisibility : CovisibilityGraph[i])
        {
            if(Covisibility.KeyFrameID == NewKeyFrameID)
            {
                Covisibility.Covisibility += Count;
                WasCovisible = true;
                break;
            }
        }

        if(!WasCovisible)
        {
            CovisibilityGraph[i].push_back(
                {
                    .KeyFrameID = NewKeyFrameID,
                    .Covisibility = Count
                });
        }

        std::sort(CovisibilityGraph[i].begin(), CovisibilityGraph[i].end(), [](const typeCovisibility& A, const typeCovisibility& B)
                {
                    return A.Covisibility > B.Covisibility;
                }
                );
    }
    std::sort(CovisibilityGraph[NewKeyFrameID].begin(), CovisibilityGraph[NewKeyFrameID].end(), [](const typeCovisibility& A, const typeCovisibility& B)
            {
                return A.Covisibility > B.Covisibility;
            }
            );
}

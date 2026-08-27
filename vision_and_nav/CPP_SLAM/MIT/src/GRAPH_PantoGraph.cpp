#include "../include/GRAPH_PantoGraph.hpp"
#include "GRAPHPriv_PantoGraph.hpp"
#include <cstring>

void GRAPH_AddKeyFrame(typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints,
        const u64 ID)
{
    assert(KeyFrame.ID == ID);

    const u64 GraphID =
        CovisibilityGraph.push_back({});

    assert(GraphID == ID);

    std::unordered_map<u64, u64>& Connections = CovisibilityGraph[KeyFrame.ID];

    Connections.clear();

    for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID =
            ImagePoint.MapPointID;

        if(MapPointID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        const typePantoMapPoint& MapPoint = GlobalMapPoints[MapPointID];

        for(const u64 OtherKeyFrameID : MapPoint.KeyFrameIDs)
        {
            if(OtherKeyFrameID == KeyFrame.ID)
            {
                continue;
            }

            if(!CovisibilityGraph.contains(OtherKeyFrameID))
            {
                continue;
            }

            ++Connections[OtherKeyFrameID];
        }
    }

    for(const auto& [OtherKeyFrameID, Count] :
        Connections)
    {
        CovisibilityGraph[OtherKeyFrameID][KeyFrame.ID] =
            Count;
    }
}

typeCovisibility GRAPH_GetMostCovisibleFrame( const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID)
{
    typeCovisibility MostCovisible
    {
        .KeyFrameID = PANTO_ID_NOT_SET,
        .Covisibility = 0
    };

    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph[KeyFrameID])
    {
        if(Count > MostCovisible.Covisibility)
        {
            MostCovisible =
            {
                .KeyFrameID = OtherKeyFrameID,
                .Covisibility = Count
            };
        }
    }

    return MostCovisible;
}

std::vector<typeCovisibility> GRAPH_GetTopNCovisibleFrames( const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID, const u64 N)
{
    std::vector<typeCovisibility> Covisibility;

    Covisibility.reserve( CovisibilityGraph[KeyFrameID].size());

    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph[KeyFrameID])
    {
        Covisibility.push_back(
        {
            .KeyFrameID = OtherKeyFrameID,
            .Covisibility = Count
        });
    }

    std::sort( Covisibility.begin(), Covisibility.end(),
            [](const typeCovisibility& A,
               const typeCovisibility& B)
            {
                return A.Covisibility >
                    B.Covisibility;
            });

    if(Covisibility.size() > N)
    {
        Covisibility.resize(N);
    }

    return Covisibility;
}


void GRAPH_UpdateCovisibility( typeCovisibilityGraph& CovisibilityGraph, const typePantoVector<typePantoMapPoint>& GlobalMapPoints, const u64 NewKeyFrameID,
        const std::vector<u64>& NewPointIDs)
{
    std::vector<u64> CovisibilityCount( CovisibilityGraph.size(), 0);

    LG_Log( LogSeverity::DBG,
            "[GRAPH_UpdateCovisibility] Covisibility graph size = %zu\n",
            CovisibilityGraph.size());

    for(const u64 MapPointID : NewPointIDs)
    {
        const typePantoMapPoint& MapPoint = GlobalMapPoints[MapPointID];

        for(const u64 KeyFrameID : MapPoint.KeyFrameIDs)
        {
            if(KeyFrameID == NewKeyFrameID)
            {
                continue;
            }

            assert(CovisibilityGraph.contains(KeyFrameID));

            ++CovisibilityCount[KeyFrameID];
        }
    }

    for(std::size_t KeyFrameID{}; KeyFrameID < CovisibilityCount.size(); KeyFrameID++)
    {
        const u64 Count = CovisibilityCount[KeyFrameID];

        if(Count == 0)
        {
            continue;
        }

        if(!CovisibilityGraph.contains(KeyFrameID))
        {
            continue;
        }

        CovisibilityGraph[NewKeyFrameID][KeyFrameID] += Count;
        CovisibilityGraph[KeyFrameID][NewKeyFrameID] += Count;
    }
}

void GRAPH_CullKeyFrame(typeCovisibilityGraph& CovisibilityGraph, u64 KeyFrameID)
{
    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph[KeyFrameID])
    {
        CovisibilityGraph[OtherKeyFrameID].erase(KeyFrameID);
    }

    CovisibilityGraph.remove(KeyFrameID);
}


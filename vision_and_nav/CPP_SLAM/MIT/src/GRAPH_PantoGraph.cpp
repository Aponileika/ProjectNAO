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

    if(!CovisibilityGraph.contains(KeyFrameID))
    {
        return MostCovisible;
    }

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

void GRAPH_DecrementAll(typeCovisibilityGraph& CovisibilityGraph, const typePantoVector<u64>& Nodes)
{
    for(std::size_t i{}; i < Nodes.size(); i++)
    {
        if(!Nodes.contains(i))
        {
            continue;
        }
        for(std::size_t j = i + 1; j < Nodes.size(); j++)
        {
            if(!Nodes.contains(j))
            {
                continue;
            }
            GRAPH_DecrementEdge(CovisibilityGraph, Nodes[i], Nodes[j]);
        }
    }
}

void GRAPH_DecrementAllOther(typeCovisibilityGraph& CovisibilityGraph, const typePantoVector<u64>& Nodes, const u64 DecrementIndex)
{
    assert(Nodes.contains(DecrementIndex));

    const u64 DecrementNode = Nodes[DecrementIndex];

    LG_Log(LogSeverity::DBG, "[GRAPH_DecrementAllOther] Decrement node index = %llu, Node = %llu", DecrementIndex, DecrementNode);

    for(std::size_t i{}; i < Nodes.size(); i++)
    {
        if(!Nodes.contains(i) || i == DecrementIndex)
        {
            continue;
        }
        
        GRAPH_DecrementEdge(CovisibilityGraph, Nodes[i], DecrementNode);
    }
}

void GRAPH_DecrementEdge(
        typeCovisibilityGraph& CovisibilityGraph,
        const u64 NodeA,
        const u64 NodeB)
{
    assert(NodeA != NodeB);

    assert(CovisibilityGraph.contains(NodeA));
    assert(CovisibilityGraph.contains(NodeB));

    assert(CovisibilityGraph[NodeA].contains(NodeB));
    assert(CovisibilityGraph[NodeB].contains(NodeA));

    u64& CovisibilityAB = CovisibilityGraph[NodeA][NodeB];

    u64& CovisibilityBA = CovisibilityGraph[NodeB][NodeA];

    assert(CovisibilityAB == CovisibilityBA);
    assert(CovisibilityAB > 0);

    CovisibilityAB--;
    CovisibilityBA--;

    if(CovisibilityAB == 0)
    {
        CovisibilityGraph[NodeA].erase(NodeB);
        CovisibilityGraph[NodeB].erase(NodeA);
    }
}

void GRAPH_Log(const typeCovisibilityGraph& CovisibilityGraph)
{
    LG_Log(
            LogSeverity::DBG,
            "[GRAPH_Log] Graph active vertices = %zu, size = %zu\n",
            CovisibilityGraph.active_size(),
            CovisibilityGraph.size());

    for(std::size_t i{}; i < CovisibilityGraph.size(); i++)
    {
        if(!CovisibilityGraph.contains(i))
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[GRAPH_Log] Vertex %zu = EMPTY\n",
                    i);

            continue;
        }

        const std::unordered_map<u64, u64>& Connections =
            CovisibilityGraph[i];

        LG_Log(
                LogSeverity::DBG,
                "[GRAPH_Log] Vertex %zu: degree = %zu\n",
                i,
                Connections.size());

        for(const auto& [OtherKeyFrameID, Count] :
            Connections)
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[GRAPH_Log]   %zu <-> %llu : %llu\n",
                    i,
                    OtherKeyFrameID,
                    Count);

            if(OtherKeyFrameID == i)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Self edge at KF %zu with count %llu\n",
                        i,
                        Count);
            }

            if(!CovisibilityGraph.contains(OtherKeyFrameID))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: KF %zu points to missing graph vertex %llu\n",
                        i,
                        OtherKeyFrameID);

                continue;
            }

            if(!CovisibilityGraph[OtherKeyFrameID].contains(i))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Asymmetric edge: %zu -> %llu exists, reverse does not\n",
                        i,
                        OtherKeyFrameID);

                continue;
            }

            const u64 ReverseCount =
                CovisibilityGraph[OtherKeyFrameID].at(i);

            if(ReverseCount != Count)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Edge weight mismatch: %zu -> %llu = %llu, reverse = %llu\n",
                        i,
                        OtherKeyFrameID,
                        Count,
                        ReverseCount);
            }

            if(Count == 0)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Zero-weight edge: %zu <-> %llu\n",
                        i,
                        OtherKeyFrameID);
            }
        }
    }
}

#include "../include/GRAPH_PantoGraph.hpp"
#include "GRAPHPriv_PantoGraph.hpp"
#include <cstring>
#include <unordered_set>

void GRAPH_AddKeyFrame(typeCovisibilityGraph* CovisibilityGraph, const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints,
        const u64 ID)
{
    CovisibilityGraph->Mutex.lock();
    assert(KeyFrame.ID == ID);

    const u64 GraphID =
        CovisibilityGraph->CovisibilityGraph.push_back({});

    assert(GraphID == ID);

    std::unordered_map<u64, u64>& Connections = CovisibilityGraph->CovisibilityGraph[KeyFrame.ID];

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

            if(!CovisibilityGraph->CovisibilityGraph.contains(OtherKeyFrameID))
            {
                continue;
            }

            ++Connections[OtherKeyFrameID];
        }
    }

    for(const auto& [OtherKeyFrameID, Count] :
        Connections)
    {
        CovisibilityGraph->CovisibilityGraph[OtherKeyFrameID][KeyFrame.ID] = Count;
    }
    CovisibilityGraph->Mutex.lock();
}

typeCovisibility GRAPH_GetMostCovisibleFrame(const typeCovisibilityGraph& CovisibilityGraph, const u64 KeyFrameID)
{
    typeCovisibility MostCovisible
    {
        .KeyFrameID = PANTO_ID_NOT_SET,
        .Covisibility = 0
    };

    if(!CovisibilityGraph.CovisibilityGraph.contains(KeyFrameID))
    {
        return MostCovisible;
    }

    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph.CovisibilityGraph[KeyFrameID])
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

    Covisibility.reserve(CovisibilityGraph.CovisibilityGraph[KeyFrameID].size());

    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph.CovisibilityGraph[KeyFrameID])
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

std::vector<typeCovisibility> GRAPH_GetTopNExternalCovisibleFrames(const typeCovisibilityGraph& CovisibilityGraph,
        const std::vector<u64>& LocalKeyFrameIDs, const u64 N, const u64 ExcludedKeyFrameID)
{
    const std::unordered_set<u64> LocalKeyFrames(LocalKeyFrameIDs.begin(), LocalKeyFrameIDs.end());
    std::unordered_map<u64, u64> ExternalCovisibility;

    for(const u64 LocalKeyFrameID : LocalKeyFrameIDs)
    {
        if(!CovisibilityGraph.CovisibilityGraph.contains(LocalKeyFrameID))
        {
            continue;
        }

        for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph.CovisibilityGraph[LocalKeyFrameID])
        {
            if(OtherKeyFrameID == ExcludedKeyFrameID || LocalKeyFrames.contains(OtherKeyFrameID) ||
               !CovisibilityGraph.CovisibilityGraph.contains(OtherKeyFrameID))
            {
                continue;
            }

            ExternalCovisibility[OtherKeyFrameID] += Count;
        }
    }

    std::vector<typeCovisibility> Result;
    Result.reserve(ExternalCovisibility.size());

    for(const auto& [KeyFrameID, Count] : ExternalCovisibility)
    {
        Result.push_back(
        {
            .KeyFrameID = KeyFrameID,
            .Covisibility = Count
        });
    }

    std::sort(Result.begin(), Result.end(),
            [](const typeCovisibility& A, const typeCovisibility& B)
            {
                if(A.Covisibility != B.Covisibility)
                {
                    return A.Covisibility > B.Covisibility;
                }

                return A.KeyFrameID < B.KeyFrameID;
            });

    if(Result.size() > N)
    {
        Result.resize(N);
    }

    return Result;
}


void GRAPH_UpdateCovisibility( typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<typePantoMapPoint>& GlobalMapPoints, const u64 NewKeyFrameID,
        const std::vector<u64>& NewPointIDs)
{
    CovisibilityGraph->Mutex.lock();
    std::vector<u64> CovisibilityCount( CovisibilityGraph->CovisibilityGraph.size(), 0);

    LG_Log( LogSeverity::DBG,
            "[GRAPH_UpdateCovisibility] Covisibility graph size = %zu\n",
            CovisibilityGraph->CovisibilityGraph.size());

    for(const u64 MapPointID : NewPointIDs)
    {
        const typePantoMapPoint& MapPoint = GlobalMapPoints[MapPointID];

        for(const u64 KeyFrameID : MapPoint.KeyFrameIDs)
        {
            if(KeyFrameID == NewKeyFrameID)
            {
                continue;
            }

            assert(CovisibilityGraph->CovisibilityGraph.contains(KeyFrameID));

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

        if(!CovisibilityGraph->CovisibilityGraph.contains(KeyFrameID))
        {
            continue;
        }

        CovisibilityGraph->CovisibilityGraph[NewKeyFrameID][KeyFrameID] += Count;
        CovisibilityGraph->CovisibilityGraph[KeyFrameID][NewKeyFrameID] += Count;
    }
    CovisibilityGraph->Mutex.unlock();
}

void GRAPH_CullKeyFrame(typeCovisibilityGraph* CovisibilityGraph, u64 KeyFrameID)
{
    CovisibilityGraph->Mutex.lock();
    for(const auto& [OtherKeyFrameID, Count] : CovisibilityGraph->CovisibilityGraph[KeyFrameID])
    {
        CovisibilityGraph->CovisibilityGraph[OtherKeyFrameID].erase(KeyFrameID);
    }

    CovisibilityGraph->CovisibilityGraph.remove(KeyFrameID);
    CovisibilityGraph->Mutex.unlock();
}

void GRAPH_DecrementAll(typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<u64>& Nodes)
{
    CovisibilityGraph->Mutex.lock();
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
            const u64 NodeA = Nodes[i];
            const u64 NodeB = Nodes[j];
            assert(NodeA != NodeB);

            assert(CovisibilityGraph->CovisibilityGraph.contains(NodeA));
            assert(CovisibilityGraph->CovisibilityGraph.contains(NodeB));

            assert(CovisibilityGraph->CovisibilityGraph[NodeA].contains(NodeB));
            assert(CovisibilityGraph->CovisibilityGraph[NodeB].contains(NodeA));

            u64* CovisibilityAB = &CovisibilityGraph->CovisibilityGraph[NodeA][NodeB];

            u64* CovisibilityBA = &CovisibilityGraph->CovisibilityGraph[NodeB][NodeA];

            assert(*CovisibilityAB == *CovisibilityBA);
            assert(*CovisibilityAB > 0);

            CovisibilityAB--;
            CovisibilityBA--;

            if(CovisibilityAB == 0)
            {
                CovisibilityGraph->CovisibilityGraph[NodeA].erase(NodeB);
                CovisibilityGraph->CovisibilityGraph[NodeB].erase(NodeA);
            }
        }
    }
    CovisibilityGraph->Mutex.unlock();
}

void GRAPH_DecrementAllOther(typeCovisibilityGraph* CovisibilityGraph, const typePantoVector<u64>& Nodes, const u64 DecrementIndex)
{
    CovisibilityGraph->Mutex.lock();
    assert(Nodes.contains(DecrementIndex));

    const u64 DecrementNode = Nodes[DecrementIndex];

    LG_Log(LogSeverity::DBG, "[GRAPH_DecrementAllOther] Decrement node index = %llu, Node = %llu", DecrementIndex, DecrementNode);

    for(std::size_t i{}; i < Nodes.size(); i++)
    {
        if(!Nodes.contains(i) || i == DecrementIndex)
        {
            continue;
        }
        
        const u64 NodeA = Nodes[i];
        const u64 NodeB = DecrementNode;
        assert(NodeA != NodeB);

        assert(CovisibilityGraph->CovisibilityGraph.contains(NodeA));
        assert(CovisibilityGraph->CovisibilityGraph.contains(NodeB));

        assert(CovisibilityGraph->CovisibilityGraph[NodeA].contains(NodeB));
        assert(CovisibilityGraph->CovisibilityGraph[NodeB].contains(NodeA));

        u64* CovisibilityAB = &CovisibilityGraph->CovisibilityGraph[NodeA][NodeB];

        u64* CovisibilityBA = &CovisibilityGraph->CovisibilityGraph[NodeB][NodeA];

        assert(*CovisibilityAB == *CovisibilityBA);
        assert(*CovisibilityAB > 0);

        CovisibilityAB--;
        CovisibilityBA--;

        if(CovisibilityAB == 0)
        {
            CovisibilityGraph->CovisibilityGraph[NodeA].erase(NodeB);
            CovisibilityGraph->CovisibilityGraph[NodeB].erase(NodeA);
        }
    }
    CovisibilityGraph->Mutex.unlock();
}

void GRAPH_DecrementEdge( typeCovisibilityGraph* CovisibilityGraph,
        const u64 NodeA, const u64 NodeB)
{
    CovisibilityGraph->Mutex.lock();
    assert(NodeA != NodeB);

    assert(CovisibilityGraph->CovisibilityGraph.contains(NodeA));
    assert(CovisibilityGraph->CovisibilityGraph.contains(NodeB));

    assert(CovisibilityGraph->CovisibilityGraph[NodeA].contains(NodeB));
    assert(CovisibilityGraph->CovisibilityGraph[NodeB].contains(NodeA));

    u64* CovisibilityAB = &CovisibilityGraph->CovisibilityGraph[NodeA][NodeB];

    u64* CovisibilityBA = &CovisibilityGraph->CovisibilityGraph[NodeB][NodeA];

    assert(*CovisibilityAB == *CovisibilityBA);
    assert(*CovisibilityAB > 0);

    CovisibilityAB--;
    CovisibilityBA--;

    if(CovisibilityAB == 0)
    {
        CovisibilityGraph->CovisibilityGraph[NodeA].erase(NodeB);
        CovisibilityGraph->CovisibilityGraph[NodeB].erase(NodeA);
    }
    CovisibilityGraph->Mutex.unlock();
}

void GRAPH_Log(const typeCovisibilityGraph& CovisibilityGraph)
{
    LG_Log(
            LogSeverity::DBG,
            "[GRAPH_Log] Graph active vertices = %zu, size = %zu\n",
            CovisibilityGraph.CovisibilityGraph.active_size(),
            CovisibilityGraph.CovisibilityGraph.size());

    for(std::size_t i{}; i < CovisibilityGraph.CovisibilityGraph.size(); i++)
    {
        if(!CovisibilityGraph.CovisibilityGraph.contains(i))
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[GRAPH_Log] Vertex %zu = EMPTY\n",
                    i);

            continue;
        }

        const std::unordered_map<u64, u64>& Connections = CovisibilityGraph.CovisibilityGraph[i];

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
                    "[GRAPH_Log]   %zu <. %llu : %llu\n",
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

            if(!CovisibilityGraph.CovisibilityGraph.contains(OtherKeyFrameID))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: KF %zu points to missing graph vertex %llu\n",
                        i,
                        OtherKeyFrameID);

                continue;
            }

            if(!CovisibilityGraph.CovisibilityGraph[OtherKeyFrameID].contains(i))
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Asymmetric edge: %zu . %llu exists, reverse does not\n",
                        i,
                        OtherKeyFrameID);

                continue;
            }

            const u64 ReverseCount =
                CovisibilityGraph.CovisibilityGraph[OtherKeyFrameID].at(i);

            if(ReverseCount != Count)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Edge weight mismatch: %zu . %llu = %llu, reverse = %llu\n",
                        i,
                        OtherKeyFrameID,
                        Count,
                        ReverseCount);
            }

            if(Count == 0)
            {
                LG_Log(
                        LogSeverity::ERROR,
                        "[GRAPH_Log] ERROR: Zero-weight edge: %zu <. %llu\n",
                        i,
                        OtherKeyFrameID);
            }
        }
    }
}

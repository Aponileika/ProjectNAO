#ifndef __MAP_MAPPING_HPP_
#define __MAP_MAPPING_HPP_
#include "IMU_PreIntegration.hpp"
#include "PT_Types.hpp"
#include <KEY_Keyframe.hpp>
#include <mutex>
#include <unordered_set>
#include "EP_CorrespondingPoints.hpp"
#include "Config.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "GRAPH_PantoGraph.hpp"
#include "FR_Frames.hpp"
#include "DBOW3_DeepBagofWords.hpp"

typedef struct
{
    typePantoVector<typeKeyFrame> KeyFrames;
    typePantoVector<typePantoMapPoint> MapPoints;
    u64 Age;
    // Incremented while Mutex is held whenever local mapping publishes a new
    // keyframe state or commits optimized state.
    u64 Revision;
    std::mutex Mutex;
}typeGlobalMap;

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
}typeLocalMapTracking;

typedef struct
{
    std::vector<typeKeyFrame> KeyFrames;
    // for imu these are pure visual constraints
    std::vector<typeKeyFrame> FixedKeyFrames;
    std::vector<typePantoMapPoint> MapPoints;
#if defined(CONFIG_IMU)
    typeNavigationState IMUAnchor;
#endif
}typeLocalMap;

typedef struct
{
    fp64 TrackedRatio;
    fp64 MedianDepth;
    std::vector<u64> VisibleMapPointIDs;
    std::vector<u64> FoundMapPointIDs;
}typeLocalMapInfo;

class typeKeyFrameQueue
{
    public:
        // naive bool can get race condition:
        // bundle adjust checks if queue empty, non empty, (a keyframe inserts in between), sets abort to false
        std::atomic<u64> KeyFrameGen{0};
        std::mutex Mutex;
        std::condition_variable QueueCV;
        std::condition_variable CompletionCV;
        std::queue<typeKeyFrame> KeyFrameQueue;
        std::atomic_bool Stop{false};
        u64 CompletedGeneration = 0;

        typeKeyFrameQueue() = default;

        u64 enque(typeKeyFrame KeyFrame)
        {
            u64 Generation;
            {
                // this makes sense, if for some reason the push fails, tracking continues as normal
                std::lock_guard<std::mutex> Lock(Mutex);
                Generation = KeyFrameGen.fetch_add(
                        1, std::memory_order_relaxed) + 1;
                KeyFrame.MappingGeneration = Generation;
                KeyFrameQueue.push(std::move(KeyFrame));
            }
            QueueCV.notify_one();
            return Generation;
        }

        bool deque(typeKeyFrame& KeyFrame)
        {
            // unique_lock since wait needs to be able to unlock and lock again.
            std::unique_lock<std::mutex> Lock(Mutex);

            QueueCV.wait(
                    Lock,
                    [this]()
                    {
                        return !KeyFrameQueue.empty() || Stop.load();
                    });

            if(KeyFrameQueue.empty())
            {
                return false;
            }

            KeyFrame = std::move(KeyFrameQueue.front());
            KeyFrameQueue.pop();

            return true;
        }

        bool PrepareForBA(u64& Generation)
        {
            std::lock_guard<std::mutex> Lock(Mutex);

            if(!KeyFrameQueue.empty())
            {
                return false;
            }

            Generation = KeyFrameGen;

            return true;
        }

        void MarkProcessed()
        {
            {
                std::lock_guard<std::mutex> Lock(Mutex);
                CompletedGeneration++;
            }
            CompletionCV.notify_all();
        }

        void WaitUntilProcessed(const u64 Generation)
        {
            std::unique_lock<std::mutex> Lock(Mutex);
            CompletionCV.wait(
                    Lock,
                    [this, Generation]()
                    {
                        return CompletedGeneration >= Generation;
                    });
        }

        void ShutDown()
        {
            Stop.store(true);
            QueueCV.notify_all();
        }

    private:
};

void MAP_InitializeFromGT(const typeNavigationState& FirstNavState, const typeNavigationState& SecondNavState,
        const typePantoFrame& FirstFrame, const typePantoFrame& SecondFrame, 
        const std::vector<typeIMUMeasurement>& IMUMeasurementsFrame1to2, typeGlobalMap* GlobalMap);
// MAP_AppendKeyFrame, MAP_CullLocalMap, MAP_CullRecentMapPoints,
// MAP_CullObservationEdges, MAP_CreateNewMapPoints, and MAP_FuseMapPoints do not lock internally.
// During multithreaded operation their caller must hold the global-map mutex
// and, where supplied, the covisibility-graph mutex as one transaction.
u64 MAP_AppendKeyFrame(typeGlobalMap* GlobalMap, const typeKeyFrame& KeyFrame);
typeLocalMapTracking MAP_CreateLocalMapTracking(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const typeKeyFrame& KeyFrame);
typeLocalMap MAP_CreateLocalMap(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph, const u64 LatestKeyFrameID);
bool MAP_CommitLocalMap(typeGlobalMap* GlobalMap,
        const typeLocalMap& LocalMap);
void MAP_CommitTrackingStatistics(typeGlobalMap* GlobalMap,
        const typeLocalMapInfo& LocalMapInfo);
typePantoVector<typePantoMapPoint> MAP_GetLastFrameMapPoints(const typeGlobalMap& Map, const typeKeyFrame& LastKeyFrame);
std::vector<typePantoMapPoint> MAP_GetLastFrameMapPoints(
        const std::vector<typePantoMapPoint>& MapPoints,
        const typeKeyFrame& LastKeyFrame);
typeLocalMapInfo MAP_MatchMapPointLocalMap(typeLocalMapTracking& LocalMap,
        typeKeyFrame& NewKeyFrame);

void MAP_CullLocalMap(typeGlobalMap* GlobalMap,
        typeCovisibilityGraph* CovisibilityGraph,
        const u64 CurrentFrameID);
void MAP_CullRecentMapPoints(typePantoVector<u64>& RecentMapPointIndexes,
        typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph);
void MAP_CullObservationEdges(typeGlobalMap* GlobalMap, typeCovisibilityGraph* CovisibilityGraph);

std::vector<u64> MAP_CreateNewMapPoints(typeGlobalMap* GlobalMap, typeKeyFrame& NewKeyFrame, typeCovisibilityGraph* CovisibilityGraph,
        const u64 LatestKeyFrameID);
std::vector<u64> MAP_FuseMapPoints(typeGlobalMap* GlobalMap, typeKeyFrame& NewKeyFrame);
void MAP_LogGlobalMapPoses(const typeGlobalMap& GlobalMap);
void MAP_LogKeyFrameProjectionError(const typeKeyFrame& KeyFrame, const typePantoVector<typePantoMapPoint>& GlobalMapPoints);
void MAP_LogGlobalMapProjectionErrors(const typeGlobalMap& GlobalMap);
void MAP_RetriangulateLOST(typeGlobalMap& GlobalMap);
u64 MAP_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame,
        std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose,
        u64* NumProjectedMapPointsOutput,
        std::vector<u64>* VisibleMapPointIDsOutput = nullptr,
        std::vector<u64>* FoundMapPointIDsOutput = nullptr);

void MAP_AssertGraphEqual(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph);
void MAP_AssertMapPointObservations(const typeGlobalMap& GlobalMap);
void MAP_LogGlobalMap(const typeGlobalMap& GlobalMap);
void MAP_LogGraphConsistency(const typeGlobalMap& GlobalMap, const typeCovisibilityGraph& CovisibilityGraph);
void MAP_LogMappingData(void);

#endif // __MAP_MAPPING_HPP_

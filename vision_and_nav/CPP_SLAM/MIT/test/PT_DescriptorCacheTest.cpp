#include "PT_PantoMapPoints.hpp"
#if !defined(PT_DESCRIPTOR_CACHE_ONLY)
#include "DENSE_DenseMapping.hpp"
#endif
#include <cstdlib>
#include <iostream>
#include <random>
#include <type_traits>
#include "../src/PANTO_Utils.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

static typeDescriptor RandomDescriptor(std::mt19937& Random)
{
    typeDescriptor Descriptor;
    for(u8& Byte : Descriptor) Byte = static_cast<u8>(Random());
    return Descriptor;
}

struct Fixture
{
    struct FrameData
    {
        u64 ID;
        typePantoKeypointFrame Points;
    };
    typePantoVector<FrameData> KeyFrames;
    std::mt19937 Random{917};

    u64 AddFrame(typeDescriptor Descriptor)
    {
        FrameData Frame{};
        Frame.ID = KeyFrames.size();
        typePantoImagePoint Point{};
        Point.Descriptor = Descriptor;
        Point.ID = 0;
        Frame.Points.ImagePoints.push_back(Point);
        return KeyFrames.push_back(std::move(Frame));
    }

    void AddObservation(typePantoMapPoint& Point, u64 Frame, u64 Image = 0)
    {
        const u64 A = Point.KeyFrameIDs.push_back(Frame);
        const u64 B = Point.ImagePointIDs.push_back(Image);
        Require(A == B, "paired sparse observation slots");
        ++Point.NumObs;
    }

    void RemoveObservation(typePantoMapPoint& Point, u64 Index)
    {
        Point.KeyFrameIDs.remove(Index);
        Point.ImagePointIDs.remove(Index);
        --Point.NumObs;
    }

    void Check(const typePantoMapPoint& Point)
    {
        std::vector<typeDescriptor> Descriptors;
        for(std::size_t i = 0; i < Point.KeyFrameIDs.size(); ++i)
        {
            Require(Point.DescriptorDistances.Observations[i].Active == Point.KeyFrameIDs.contains(i),
                    "cache activity matches sparse observations");
            if(Point.KeyFrameIDs.contains(i))
            {
                Descriptors.push_back(KeyFrames[Point.KeyFrameIDs[i]].Points.ImagePoints[Point.ImagePointIDs[i]].Descriptor);
                for(std::size_t j = 0; j < i; ++j)
                {
                    if(Point.KeyFrameIDs.contains(j))
                    {
                        const auto& A = KeyFrames[Point.KeyFrameIDs[i]].Points.ImagePoints[Point.ImagePointIDs[i]].Descriptor;
                        const auto& B = KeyFrames[Point.KeyFrameIDs[j]].Points.ImagePoints[Point.ImagePointIDs[j]].Descriptor;
                        Require(Point.DescriptorDistances.Distance(i, j) == PANTO_HammingDistance(A, B),
                                "cached distance equals direct Hamming distance");
                    }
                }
            }
        }
        if(!Descriptors.empty())
        {
            std::vector<i32> Scratch;
            Require(Point.Descriptor == PT_CalculateNewDescriptor(Descriptors, Scratch),
                    "cached representative equals original smallest-median rule");
        }
    }
};

static void TestInsertionRemovalAndReusedSlots()
{
    Fixture F;
    typePantoMapPoint Point{};
    for(u64 Count = 0; Count < 32; ++Count)
    {
        F.AddObservation(Point, F.AddFrame(RandomDescriptor(F.Random)));
        Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == Count, "adding one observation evaluates only new pairs");
        F.Check(Point);
        Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == 0, "unchanged observations reuse every distance");
    }
    for(const u64 Index : {1, 3, 7, 12})
    {
        F.RemoveObservation(Point, Index);
        Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == 0, "removing an observation needs no new distances");
        F.Check(Point);
    }
    for(u64 i = 0; i < 4; ++i)
    {
        const u64 OldCount = Point.KeyFrameIDs.active_size();
        F.AddObservation(Point, F.AddFrame(RandomDescriptor(F.Random)));
        Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == OldCount, "recycled observation slot invalidates only its pairs");
        F.Check(Point);
    }
    F.KeyFrames[Point.KeyFrameIDs[0]].Points.ImagePoints[0].Descriptor = RandomDescriptor(F.Random);
    Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == 31, "changed descriptor invalidates its row and column");
    F.Check(Point);

    typePantoImagePoint NewImage{};
    NewImage.Descriptor = RandomDescriptor(F.Random);
    NewImage.ID = F.KeyFrames[Point.KeyFrameIDs[0]].Points.ImagePoints.push_back(NewImage);
    Point.ImagePointIDs[0] = NewImage.ID;
    Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == 31, "changed image identity cannot reuse stale pairs");
    F.Check(Point);
    for(std::size_t i = 0; i < Point.KeyFrameIDs.size(); ++i) F.RemoveObservation(Point, i);
    Require(PT_UpdateMapPointDescriptor(Point, F.KeyFrames) == 0 && Point.DescriptorDistances.ActiveIndices.empty(),
            "removing all observations leaves an inactive cache");
    F.Check(Point);
    Point.DescriptorDistances.Clear();
    Require(Point.DescriptorDistances.Distances.capacity() == 0, "deleting a point releases cache storage");
}

static void TestFusionReuseAndRepresentative()
{
    Fixture F;
    typePantoMapPoint A{}, B{};
    typeDescriptor Ones, Zeros{};
    Ones.fill(255);
    for(u64 i = 0; i < 3; ++i) F.AddObservation(A, F.AddFrame(Ones));
    for(u64 i = 0; i < 5; ++i) F.AddObservation(B, F.AddFrame(Zeros));
    Require(PT_UpdateMapPointDescriptor(A, F.KeyFrames) == 3, "initial A cache has one distance per pair");
    Require(PT_UpdateMapPointDescriptor(B, F.KeyFrames) == 10, "initial B cache has one distance per pair");
    for(const u64 Frame : B.KeyFrameIDs) F.AddObservation(A, Frame);
    Require(PT_UpdateMapPointDescriptor(A, F.KeyFrames, &B.DescriptorDistances) == 15,
            "fusion evaluates A-B pairs and reuses both internal blocks");
    Require(A.Descriptor == Zeros, "fusion can select the absorbed point's observed descriptor");
    Require(A.DescriptorDistances.Distance(0, 3) == PANTO_DESCRIPTOR_SIZE * 8, "full-width Hamming distance is not truncated");
    F.Check(A);
    F.Check(B);
}

static void TestFusionWithConflictingAndRemovedObservations()
{
    Fixture F;
    typePantoMapPoint A{}, B{};
    const u64 Shared = F.AddFrame(RandomDescriptor(F.Random));
    typePantoImagePoint Conflict{};
    Conflict.Descriptor = RandomDescriptor(F.Random);
    F.KeyFrames[Shared].Points.ImagePoints.push_back(Conflict);
    F.AddObservation(A, Shared);
    F.AddObservation(B, Shared, 1);
    for(u64 i = 0; i < 2; ++i) F.AddObservation(A, F.AddFrame(RandomDescriptor(F.Random)));
    for(u64 i = 0; i < 3; ++i) F.AddObservation(B, F.AddFrame(RandomDescriptor(F.Random)));
    PT_UpdateMapPointDescriptor(A, F.KeyFrames);
    PT_UpdateMapPointDescriptor(B, F.KeyFrames);
    F.RemoveObservation(B, 2);
    PT_UpdateMapPointDescriptor(B, F.KeyFrames);
    // The real fusion path retains the survivor's observation in shared KFs.
    for(std::size_t i = 1; i < B.KeyFrameIDs.size(); ++i)
    {
        if(B.KeyFrameIDs.contains(i)) F.AddObservation(A, B.KeyFrameIDs[i], B.ImagePointIDs[i]);
    }
    Require(PT_UpdateMapPointDescriptor(A, F.KeyFrames, &B.DescriptorDistances) == 6,
            "fusion excludes deleted/conflicting observations from reused distances");
    F.Check(A);
}

static void TestSnapshotsAndTies()
{
    static_assert(std::is_nothrow_move_constructible_v<typePantoMapPoint>);
    Fixture F;
    typePantoMapPoint A{};
    for(u64 i = 0; i < 6; ++i) F.AddObservation(A, F.AddFrame(RandomDescriptor(F.Random)));
    PT_UpdateMapPointDescriptor(A, F.KeyFrames);
    typePantoMapPoint Copy = A;
    Require(Copy.Descriptor == A.Descriptor && Copy.DescriptorDistances.Observations.empty(),
            "tracking snapshots copy the representative without the quadratic cache");
    Require(PT_UpdateMapPointDescriptor(Copy, F.KeyFrames) == 15, "modified copies rebuild an independent cache");
    F.Check(A);
    const auto* Rows = A.DescriptorDistances.Distances.data();
    typePantoMapPoint Moved = std::move(A);
    Require(Moved.DescriptorDistances.Distances.data() == Rows, "moving live map points retains their cache");
    F.Check(Moved);

    typePantoMapPoint Pair{};
    typeDescriptor Zero{}, One{};
    One[0] = 1;
    F.AddObservation(Pair, F.AddFrame(Zero));
    F.AddObservation(Pair, F.AddFrame(One));
    PT_UpdateMapPointDescriptor(Pair, F.KeyFrames);
    Require(Pair.Descriptor == Zero, "equal medians retain the first active observation");
    F.RemoveObservation(Pair, 0);
    PT_UpdateMapPointDescriptor(Pair, F.KeyFrames);
    Require(Pair.Descriptor == One, "removing the representative selects a surviving descriptor");
}

#if !defined(PT_DESCRIPTOR_CACHE_ONLY)
static void TestDensePoseQueue()
{
    typeLocalMap Map{};
    for(u64 i = 0; i < 4; ++i)
    {
        typeKeyFrame Frame{};
        Frame.ID = 17 + i;
        Frame.Camera.Pose = typeCameraPose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(i, -2.0 * i, 0.5));
        Frame.Points.ImagePoints = typePantoVector<typePantoImagePoint>(1000);
        if(i < 2) Map.KeyFrames.push_back(std::move(Frame));
        else Map.FixedKeyFrames.push_back(std::move(Frame));
    }
    cv::Mat Left(3, 4, CV_8U, cv::Scalar(7));
    cv::Mat Right(3, 4, CV_8U, cv::Scalar(9));
    auto Payload = DENSE_CreateLocalMapData({17, Left, Right}, Map);
    Require(Payload.LocalKeyFramePoses.size() == 4, "dense payload includes adjustable and fixed poses");
    Require(sizeof(typeDenseKeyFramePose) < sizeof(typeKeyFrame), "dense entries contain no full keyframe");
    const auto* Poses = Payload.LocalKeyFramePoses.data();
    typeSPSCQueue<typeDenseLocaMapData> Queue;
    Queue.enque(std::move(Payload));
    typeDenseLocaMapData Received;
    Require(Queue.deque(Received) && Received.LocalKeyFramePoses.data() == Poses, "dense queue moves the pose vector");
    Require(Received.DenseData.LeftImage.data == Left.data && Received.DenseData.RightImage.data == Right.data,
            "stereo images remain shared through cv::Mat ownership");
    for(u64 i = 0; i < 4; ++i)
    {
        const auto& Pose = Received.LocalKeyFramePoses[i];
        Require(Pose.KeyFrameID == 17 + i && Pose.Pose.t.isApprox(Eigen::Vector3d(i, -2.0 * i, 0.5)),
                "dense poses retain IDs, ordering and transforms");
    }
    Map.KeyFrames[0].Camera.Pose.t.setZero();
    Require(Received.LocalKeyFramePoses[0].Pose.t.z() == 0.5, "queued poses are independent snapshots");
}
#endif

int main()
{
    TestInsertionRemovalAndReusedSlots();
    TestFusionReuseAndRepresentative();
    TestFusionWithConflictingAndRemovedObservations();
    TestSnapshotsAndTies();
#if !defined(PT_DESCRIPTOR_CACHE_ONLY)
    TestDensePoseQueue();
    std::cout << "Descriptor-distance cache and dense-pose queue tests passed\n";
#else
    std::cout << "Descriptor-distance cache sanitizer tests passed\n";
#endif
}

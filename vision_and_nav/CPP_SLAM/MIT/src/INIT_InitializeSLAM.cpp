#include "../include/INIT_InitializeSLAM.hpp"
#include "INITPriv_InitializeSLAM.hpp"

static typePantoInitData InitData = 
{
    .InitFrames{},
    .FeatureTracks{},
    .EnoughStationaryPointsForInit = false
};

DBoW3::Vocabulary Vocabulary; 

void INIT_CreateInitData(void)
{
    Vocabulary = DBOW3_GetVocabulary();

    typePantoFrame FirstFrame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(FirstFrame.Frame);
    INITPriv_AppendFrame(Descriptors.Points, Descriptors.Descriptors, FirstFrame.TimeStamp);
}

typeInitReconstruction INIT_ProcessNewFrame(void)
{
    typePantoFrame Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);
    INITPriv_AppendFrame(Descriptors.Points, Descriptors.Descriptors, Frame.TimeStamp);
    INITPriv_MatchHistoricalFrames();
    std::vector<u64> StationaryTrackIDs = INITPriv_STRANSAC();

    if(StationaryTrackIDs.size() < PANTO_INIT_MIN_STATIONARY_POINTS)
    {
        return 
        {
            .R{},
            .t{}, 
            .NumPointsInFront{},
            .MapPoints{}, 
            .ChosenInitFrameID{},
            .Valid = false
        };
    }

    std::vector<u64> CandidateIDs = INITPriv_GetCandidateFrameIDs(StationaryTrackIDs);

    std::vector<std::thread> ReconstructionThreads;
    ReconstructionThreads.reserve(PANTO_NUM_THREADS_MAX);

    std::vector<typeInitReconstruction> Reconstruction(PANTO_NUM_THREADS_MAX);

    for(u64 i = 0; i < PANTO_NUM_THREADS_MAX; i++)
    {
        ReconstructionThreads.emplace_back([&, i]()
        {
            Reconstruction[i] = INITPriv_Reconstruct(InitData.InitFrames[CandidateIDs[i]],
                    InitData.InitFrames.back(),
                    StationaryTrackIDs);
        });
    }

    for(u64 i = 0; i < PANTO_NUM_THREADS_MAX; i++)
    {
        ReconstructionThreads[i].join();
    }

    std::size_t MostPoints = 0;
    u64 BestReconstructionIndex = 0;
    for(u64 i = 0; i < PANTO_NUM_THREADS_MAX; i++)
    {
        std::size_t NumPoints = Reconstruction[i].MapPoints.size();
        if(NumPoints > MostPoints)
        {
            BestReconstructionIndex = i;
        }
    }
    
    return Reconstruction[BestReconstructionIndex];
}

void INIT_DestroyInitData(void)
{
    InitData.InitFrames.resize(0);
    InitData.FeatureTracks.resize(0);
}

typeGlobalMap INIT_ConstructInitialMap(typeInitReconstruction Reconstruction)
{
    assert(Reconstruction.Valid == true);
    assert(Reconstruction.MapPoints.size() > 0);
    assert(Reconstruction.NumPointsInFront > 0);

    const u64 FirstFrameID = Reconstruction.ChosenInitFrameID.first;
    const u64 SecondFrameID = Reconstruction.ChosenInitFrameID.second;

    std::vector<typePantoMapPoint> InitialMapPoints;
    InitialMapPoints.reserve(Reconstruction.MapPoints.size());

    for(std::size_t i{}; i < Reconstruction.MapPoints.size(); i++)
    {
        const typeInitMapPoint& InitMapPoint = Reconstruction.MapPoints[i];

        std::vector<u64> ImagePointIDs(2, 0);
        ImagePointIDs[0] = InitMapPoint.InitImagePointID.first;
        ImagePointIDs[1] = InitMapPoint.InitImagePointID.second;

        typeDescriptor Descriptor = InitData.InitFrames[SecondFrameID].ImagePoints[ImagePointIDs[1]].Descriptor;

        std::vector<u64> KeyFrameIDs(2, 0);
        KeyFrameIDs[0] = FirstFrameID;
        KeyFrameIDs[1] = SecondFrameID;

        InitialMapPoints.push_back(
            {
                .Point = InitMapPoint.Point4D,
                .Descriptor = Descriptor,
                .KeyFrameIDs = KeyFrameIDs,
                .ImagePointIDs = ImagePointIDs,
                .ID = static_cast<u64>(i)
            });
    }

    std::vector<typePantoKeypointFrame> ImagePoints;
    ImagePoints.reserve(2);

    ImagePoints.push_back(INITPriv_GetKeyPointFrame(FirstFrameID));
    ImagePoints.push_back(INITPriv_GetKeyPointFrame(SecondFrameID));


    Eigen::Matrix3d FirstR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Firstt(0, 0, 0);
    typeCamera FirstCamera = CM_CreateCam(FirstR, Firstt, 0, InitData.InitFrames[FirstFrameID].TimeStamp);

    Eigen::Matrix3d SecondR = Reconstruction.R;
    Eigen::Vector3d Secondt = Reconstruction.t;
    typeCamera SecondCamera = CM_CreateCam(SecondR, Secondt, 1, InitData.InitFrames[SecondFrameID].TimeStamp);

    std::vector<typeKeyFrame> KeyFrames;
    KeyFrames.push_back(
        {
            .Points = ImagePoints[0],
            .BowVector = InitData.InitFrames[FirstFrameID].BoWVector,
            .FeatureVector = InitData.InitFrames[FirstFrameID].FeatureVector,
            .Pose = FirstCamera,
            .ID = 0
        });

    KeyFrames.push_back(
        {
            .Points = ImagePoints[1],
            .BowVector = InitData.InitFrames[SecondFrameID].BoWVector,
            .FeatureVector = InitData.InitFrames[SecondFrameID].FeatureVector,
            .Pose = SecondCamera,
            .ID = 1
        });
    
    return {.KeyFrames = KeyFrames, .MapPoints = InitialMapPoints};
}

typePantoKeypointFrame INITPriv_GetKeyPointFrame(u64 InitFrameID)
{
    typePantoKeypointFrame Ret{};

    const typeInitFrame& Frame = InitData.InitFrames[InitFrameID];

    const u64 NumImagePoints = Frame.ImagePoints.size();

    for(std::size_t i{}; i < NumImagePoints; i++)
    {
        const typeInitImagePoint& InitImagePoint = Frame.ImagePoints[i];

        Eigen::Vector2d Point = InitImagePoint.Point;
        u64 CellX = static_cast<u64>(Point[0]) / PANTO_CELL_SIZE;
        u64 CellY = static_cast<u64>(Point[1]) / PANTO_CELL_SIZE;

        u64 CellIndex = CellY * PANTO_GRID_COLUMNS + CellX;

        typeDescriptor Descriptor = InitImagePoint.Descriptor;

        typePantoImagePoint CandidateImagePoint = 
        {
            .Point = Point,
            .Descriptor = Descriptor,
            .MapPointID = PANTO_ID_NOT_SET,
            .ID = static_cast<u64>(i),
            .CellID = CellIndex
        };

        Ret.ImagePoints.push_back(CandidateImagePoint);
        Ret.CellIndexingArray[CellIndex].push_back(i);
    }

    return Ret;
}

/** 
 *
 * Updates feature tracks.
 * */
void INITPriv_MatchHistoricalFrames(void)
{
    assert(!InitData.InitFrames.empty());
    typeInitFrame& LatestFrame = InitData.InitFrames.back();
    const DBoW3::FeatureVector& FeatureVectorNew = LatestFrame.FeatureVector;
    for(typeInitFrame& HistoricalFrame : std::views::reverse(InitData.InitFrames))
    {
        if(HistoricalFrame.ID == LatestFrame.ID)
        {
            continue;
        }
        const DBoW3::FeatureVector& FeatureVectorHistorical = HistoricalFrame.FeatureVector;

        auto FeatureIteratorNew = FeatureVectorNew.begin();
        auto FeatureIteratorHistorical = FeatureVectorHistorical.begin();

        while(FeatureIteratorNew != FeatureVectorNew.end() && FeatureIteratorHistorical != FeatureVectorHistorical.end())
        {
            if(FeatureIteratorNew->first == FeatureIteratorHistorical->first)
            {
                //Feature vector match
                const std::vector<u32>& FeatureIDsNew = FeatureIteratorNew->second;
                const std::vector<u32>& FeatureIDsHistorical = FeatureIteratorHistorical->second;

                for(const u32& FeatureIDNew : FeatureIDsNew)
                {
                    typeInitImagePoint& ImagePointNew = LatestFrame.ImagePoints[FeatureIDNew];

                    if(ImagePointNew.FeatureTrackID != PANTO_ID_NOT_SET)
                    {
                        continue;
                    }

                    u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                    u32 SecondBestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                    u64 BestFeatureID = PANTO_ID_NOT_SET;

                    for(const u32& FeatureIDHistorical : FeatureIDsHistorical)
                    {
                        typeInitImagePoint& ImagePointHistorical = HistoricalFrame.ImagePoints[FeatureIDHistorical];

                        u32 HammingDistance = PANTO_HammingDistance(ImagePointNew.Descriptor, ImagePointHistorical.Descriptor);

                        if(HammingDistance < BestDistance)
                        {
                            BestDistance = HammingDistance;
                            BestFeatureID = FeatureIDHistorical;
                            SecondBestDistance = BestDistance;
                        }
                    }
                    if(SecondBestDistance == PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1) continue;
                    if((static_cast<fp64>(BestDistance) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestDistance))
                            && (BestDistance < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD))
                    {
                        const typeInitImagePoint& TopCandidate = HistoricalFrame.ImagePoints[BestFeatureID];
                        const u64 FeatureTrackID = TopCandidate.FeatureTrackID;

                        if(FeatureTrackID == PANTO_ID_NOT_SET)
                        {
                            std::vector<u64> NewTrack(InitData.InitFrames.size(), PANTO_ID_NOT_SET);
                            *(NewTrack.rbegin()) = TopCandidate.ID;
                            *(NewTrack.rbegin() - 1) = ImagePointNew.ID;
                            typeFeatureTrack FeatureTrack = 
                            {
                                .FeatureTrack = NewTrack,
                                .InlierCount = 0,
                                .OutlierCount = 0
                            };

                            InitData.FeatureTracks.push_back(FeatureTrack);
                        }
                        else
                        {
                            InitData.FeatureTracks[FeatureTrackID].FeatureTrack.push_back(ImagePointNew.ID);
                        }
                    }
                }
                ++FeatureIteratorNew;
                ++FeatureIteratorHistorical;
            }
            else if(FeatureIteratorNew->first < FeatureIteratorHistorical->first)
            {
                FeatureIteratorNew = FeatureVectorNew.lower_bound(FeatureIteratorHistorical->first);
            }
            else
            {
                FeatureIteratorHistorical = FeatureVectorHistorical.lower_bound(FeatureIteratorNew->first);
            }
        }
    }
}

std::vector<u64> INITPriv_STRANSAC(void)
{
    const typeInitFrame& LatestFrame = InitData.InitFrames.back();

    // Get all feature tracks from the newest frame
    std::vector<u64> LatestFrameFeatureTrackIDs;

    LatestFrameFeatureTrackIDs.reserve(LatestFrame.ImagePoints.size());

    for(const typeInitImagePoint& ImagePoint : LatestFrame.ImagePoints) 
    {
        const u64 FeatureTrackID = ImagePoint.FeatureTrackID;
        if(FeatureTrackID == PANTO_ID_NOT_SET)
        {
            continue;
        }
        
        LatestFrameFeatureTrackIDs.push_back(FeatureTrackID);
    }

    for(typeInitFrame& HistoricalFrame : InitData.InitFrames)
    {

        const u64 HistoricalFrameID = HistoricalFrame.ID;
        if(HistoricalFrameID == LatestFrame.ID)
        {
            continue;
        }

        // Get all feature tracks from the newest frame
        std::vector<u64> HistoricalFrameFeatureTrackIDs;
        HistoricalFrameFeatureTrackIDs.reserve(LatestFrame.ImagePoints.size());

        std::vector<u64> CommonFeatureTrackIDs;
        CommonFeatureTrackIDs.reserve(LatestFrame.ImagePoints.size());

        std::vector<Eigen::Vector2d> LatestFramePoints;
        LatestFramePoints.reserve(LatestFrame.ImagePoints.size());

        std::vector<Eigen::Vector2d> HistoricalFramePoints;
        HistoricalFramePoints.reserve(LatestFrame.ImagePoints.size());

        for(const u64 TrackID : LatestFrameFeatureTrackIDs) 
        {
            const u64 HistoricalFeatureID = InitData.FeatureTracks[TrackID].FeatureTrack[HistoricalFrameID];

            if(HistoricalFeatureID == PANTO_ID_NOT_SET)
            {
                continue;
            }
            
            const u64 NewFeatureID = InitData.FeatureTracks[TrackID].FeatureTrack[LatestFrame.ID];

            CommonFeatureTrackIDs.push_back(TrackID);

            LatestFramePoints.push_back(LatestFrame.ImagePoints[NewFeatureID].Point);
            HistoricalFramePoints.push_back(HistoricalFrame.ImagePoints[HistoricalFeatureID].Point);
        }
        std::unique_ptr<ImageToImageMapping> Mapping = INITPriv_ScoredFAndHEstimation(LatestFramePoints, HistoricalFramePoints);

        std::vector<fp64> Error = Mapping->Error(LatestFramePoints, HistoricalFramePoints);

        const fp64 MappingErrorThreshold = Mapping->GetErrorThreshold();
        for(std::size_t i{}; i < Error.size(); i++)
        {
            typeFeatureTrack& FeatureTrack = InitData.FeatureTracks[CommonFeatureTrackIDs[i]];

            if(Error[i] < MappingErrorThreshold)
            {
                ++FeatureTrack.InlierCount;
            }
            else
            {
                ++FeatureTrack.OutlierCount;
            }
        }
    }

    std::vector<u64> StationaryTrackIDs;
    StationaryTrackIDs.reserve(LatestFrame.ImagePoints.size());
    for(const typeInitImagePoint& NewImagePoint : LatestFrame.ImagePoints)
    {
        const u64 FeatureTrackID = NewImagePoint.FeatureTrackID;
        if(FeatureTrackID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        const typeFeatureTrack& FeatureTrack = InitData.FeatureTracks[FeatureTrackID];
        const u64 Total = FeatureTrack.InlierCount + FeatureTrack.OutlierCount;
        if(Total == 0)
        {
            continue;
        }

        const fp64 Ratio = static_cast<fp64>(FeatureTrack.InlierCount) / static_cast<fp64>(Total);
        if(Ratio > PANTO_INIT_STRANSAC_RATIO_INLIER_OUTLIER_THRESHOLD)
        {
            StationaryTrackIDs.push_back(NewImagePoint.FeatureTrackID);
        }
    }

    return StationaryTrackIDs;
}

std::unique_ptr<ImageToImageMapping> INITPriv_ScoredFAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical)
{
    std::unique_ptr<FundamentalMatrixMapping> Fundamental = std::make_unique<FundamentalMatrixMapping>();
    std::unique_ptr<HomographyMapping> Homography = std::make_unique<HomographyMapping>();

    const u64 Seed = INITPriv_RandomSeed();

    std::thread HomographyThread(&HomographyMapping::Estimate, Homography.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical), Seed);
    std::thread FundamentalThread(&FundamentalMatrixMapping::Estimate, Fundamental.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical), Seed);

    HomographyThread.join();
    FundamentalThread.join();

    if(Fundamental->MaxScore > Homography->MaxScore)
    {
        return Fundamental;
    }

    return Homography;
}

u64 INITPriv_RandomSeed(void)
{
    std::random_device RandomDevice;

    u64 High = static_cast<u64>(RandomDevice());
    u64 Low  = static_cast<u64>(RandomDevice());

    return (High << 32) | Low;
}

std::vector<u64> INITPriv_GetCandidateFrameIDs(std::vector<u64> StationaryTrackIDs)
{
    std::vector<u64> Covisibility(InitData.InitFrames.size(), 0);

    const u64 LatestFrameID = InitData.InitFrames.back().ID;
    for(const u64 TrackID : StationaryTrackIDs)
    {
        const typeFeatureTrack& FeatureTrack = InitData.FeatureTracks[TrackID];

        for(u64 i = 0; i < LatestFrameID; i++)
        {
            if(FeatureTrack.FeatureTrack[i] != PANTO_ID_NOT_SET)
            {
                Covisibility[i]++;
            }
        }
    }

    std::vector<u64> CandidateFrameIDs(LatestFrameID);

    std::iota(CandidateFrameIDs.begin(), CandidateFrameIDs.end(), 0);
    std::sort(CandidateFrameIDs.begin(), CandidateFrameIDs.end(), [&Covisibility](const u64 A, const u64 B)
        {
            return Covisibility[A] > Covisibility[B];
        });

    const std::size_t NumCandidates = std::min<std::size_t>(PANTO_NUM_THREADS_MAX, CandidateFrameIDs.size());

    CandidateFrameIDs.resize(NumCandidates);

    return CandidateFrameIDs;
}

typeInitReconstruction INITPriv_Reconstruct( const typeInitFrame& HistoricalFrame, const typeInitFrame& NewFrame, const std::vector<u64>& StationaryTrackIDs)
{
    std::vector<Eigen::Vector2d> HistoricalPoints;
    std::vector<Eigen::Vector2d> NewPoints;

    HistoricalPoints.reserve(StationaryTrackIDs.size());
    NewPoints.reserve(StationaryTrackIDs.size());
    std::vector<std::pair<u64, u64>> ImagePointIDs;

    for(const u64 TrackID : StationaryTrackIDs)
    {
        const typeFeatureTrack& FeatureTrack = InitData.FeatureTracks[TrackID];

        const u64 HistoricalFeatureID = FeatureTrack.FeatureTrack[HistoricalFrame.ID];
        const u64 NewFeatureID = FeatureTrack.FeatureTrack[NewFrame.ID];

        if(HistoricalFeatureID == PANTO_ID_NOT_SET || NewFeatureID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        HistoricalPoints.push_back( HistoricalFrame.ImagePoints[HistoricalFeatureID].Point);
        NewPoints.push_back( NewFrame.ImagePoints[NewFeatureID].Point);
        ImagePointIDs.push_back(
        {
            HistoricalFeatureID,
            NewFeatureID
        });
    }

    if(HistoricalPoints.size() < PANTO_FUNDAMENTAL_MIN_POINTS)
    {
        return {};
    }

    /*
     * Re-estimate F/H using only the stationary correspondences.
     *
     * Convention here:
     *
     *   Point1 = Historical
     *   Point2 = New
     */
    std::unique_ptr<ImageToImageMapping> Mapping = INITPriv_ScoredFAndHEstimation(HistoricalPoints, NewPoints);

    const typeCameraIntrinsics* Intrinsics = CM_GetIntrinsics();

    const Eigen::Matrix3d& K = Intrinsics->K;

    /*
     * Relative pose from historical camera -> new camera:
     *
     * X_new = R * X_historical + t
     */

    const std::pair<u64, u64> FrameIDs =
    {
        HistoricalFrame.ID,
        NewFrame.ID
    };

    typeInitReconstruction Reconstruction =
        Mapping->Reconstruct(
            HistoricalPoints,
            NewPoints,
            ImagePointIDs,
            FrameIDs,
            K);

    return Reconstruction;
}

void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp)
{
    std::vector<typeInitFrame>& InitFrames = InitData.InitFrames;

    typeInitFrame InitFrame{};

    std::vector<cv::Mat> DescriptorVector;
    DescriptorVector.reserve(Descriptors.rows);
    for(i32 i{}; i < Descriptors.rows; i++)
    {
        DescriptorVector.push_back(Descriptors.row(i));
    }

    const i32 Levels = PANTO_DBOW_LEVELSUP;

    Vocabulary.transform(DescriptorVector, InitFrame.BoWVector, InitFrame.FeatureVector, Levels);

    const std::size_t NPoints = Points.size();

    InitFrame.ImagePoints.reserve(NPoints);

    for(std::size_t i{}; i < NPoints; i++)
    {
        Eigen::Vector2d Point(Points[i].x, Points[i].y);
        typeDescriptor Descriptor;

        std::memcpy(Descriptor.data(), Descriptors.ptr<u8>(i), PANTO_DESCRIPTOR_SIZE);

        InitFrame.ImagePoints.push_back(
        {
            .Point = Point,
            .Descriptor = Descriptor,
            .ID = static_cast<u64>(i),
            .FeatureTrackID = PANTO_ID_NOT_SET
        });
    }

    InitFrame.ID = static_cast<u64>(InitFrames.size());
    InitFrame.TimeStamp = TimeStamp;
    InitFrames.push_back(InitFrame);
}

#include "../include/INIT_InitializeSLAM.hpp"
#include "INITPriv_InitializeSLAM.hpp"

static typePantoInitData InitData = 
{
    .InitFrames{},
    .FeatureTracks{},
    .EnoughStationaryPointsForInit = false
};

static u32 InitNumFrames = 1;

void INIT_CreateInitData(void)
{
    InitData =
    {
        .InitFrames{},
        .FeatureTracks{},
        .EnoughStationaryPointsForInit = false
    };
    InitNumFrames = 1;

    LG_Log(LogSeverity::DBG, "[INIT_CreateInitData] Creating initialization data\n");

    typePantoFrame FirstFrame = FR_GetFrame();

    LG_Log(LogSeverity::DBG, "[INIT_CreateInitData] Extracting descriptors from first frame\n");
    DescRet Descriptors = EP_GetDescriptors(FirstFrame.Frame);

    LG_Log(LogSeverity::DBG, "[INIT_CreateInitData] Appending first initialization frame\n");
    INITPriv_AppendFrame(
        Descriptors.Points,
        Descriptors.Descriptors,
        FirstFrame.TimeStamp,
        FirstFrame.Path);

    LG_Log(LogSeverity::DBG, "[INIT_CreateInitData] Initialization data created\n");
}

typeInitReconstruction INIT_ProcessNewFrame(void)
{
    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Processing new initialization frame\n");

    typePantoFrame Frame = FR_GetFrame();

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Extracting descriptors\n");
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Appending initialization frame\n");
    INITPriv_AppendFrame(
        Descriptors.Points,
        Descriptors.Descriptors,
        Frame.TimeStamp,
        Frame.Path);

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Matching historical frames\n");
    INITPriv_MatchHistoricalFrames();

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Running ST-RANSAC\n");
    std::vector<u64> StationaryTrackIDs = INITPriv_STRANSAC();

    LG_Log(
        LogSeverity::DBG,
        "[INIT_ProcessNewFrame] ST-RANSAC found %zu stationary tracks\n",
        StationaryTrackIDs.size());

    InitNumFrames++;

    if(StationaryTrackIDs.size() < PANTO_INIT_MIN_STATIONARY_POINTS || InitNumFrames < PANTO_INIT_MIN_NUM_FRAMES)
    {
        LG_Log(
            LogSeverity::DBG,
            "[INIT_ProcessNewFrame] Not enough stationary tracks for reconstruction, %zu\n",
            StationaryTrackIDs.size());

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

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Finding candidate initialization frames\n");
    std::vector<u64> CandidateIDs =
        INITPriv_GetCandidateFrameIDs(StationaryTrackIDs);

    const std::size_t NumCandidates = CandidateIDs.size();

    if(NumCandidates == 0)
    {
        LG_Log(
            LogSeverity::DBG,
            "[INIT_ProcessNewFrame] No candidate initialization frames\n");

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

    LG_Log(
        LogSeverity::DBG,
        "[INIT_ProcessNewFrame] Found %zu candidate initialization frames\n",
        CandidateIDs.size());

    std::vector<typeInitReconstruction> Reconstruction(NumCandidates);

    for(std::size_t BatchStart = 0; BatchStart < NumCandidates; BatchStart += PANTO_NUM_THREADS_MAX)
    {
        const std::size_t BatchEnd =
            std::min<std::size_t>(BatchStart + PANTO_NUM_THREADS_MAX, NumCandidates);

        std::vector<std::thread> ReconstructionThreads;
        ReconstructionThreads.reserve(BatchEnd - BatchStart);

        LG_Log(
            LogSeverity::DBG,
            "[INIT_ProcessNewFrame] Starting reconstruction batch [%zu, %zu)\n",
            BatchStart,
            BatchEnd);

        for(std::size_t i = BatchStart; i < BatchEnd; i++)
        {
            ReconstructionThreads.emplace_back([&, i]()
            {
                LG_Log(
                    LogSeverity::DBG,
                    "[INIT_ProcessNewFrame] Reconstruction thread %zu started for frame %llu\n",
                    i,
                    static_cast<unsigned long long>(CandidateIDs[i]));

                Reconstruction[i] = INITPriv_Reconstruct(
                        InitData.InitFrames[CandidateIDs[i]],
                        InitData.InitFrames.back(),
                        StationaryTrackIDs);
            });
        }

        LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Joining reconstruction batch threads\n");

        for(std::thread& ReconstructionThread : ReconstructionThreads)
        {
            ReconstructionThread.join();
        }

        LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] Reconstruction batch complete\n");
    }

    LG_Log(LogSeverity::DBG, "[INIT_ProcessNewFrame] All reconstruction batches complete\n");

    std::size_t MostPoints = 0;
    u64 BestReconstructionIndex = 0;

    for(u64 i = 0; i < NumCandidates; i++)
    {
        const std::size_t NumPoints =
            Reconstruction[i].MapPoints.size();

        LG_Log(LogSeverity::DBG,
            "[INIT_ProcessNewFrame] Reconstruction %llu has %zu map points\n",
            static_cast<unsigned long long>(i),
            NumPoints);

        if(NumPoints > MostPoints)
        {
            MostPoints = NumPoints;
            BestReconstructionIndex = i;
        }
    }

    if(MostPoints < PANTO_MIN_NUMBER_INITIAL_MAP_POINTS)
    {
        Reconstruction[BestReconstructionIndex].Valid = false;
        return Reconstruction[BestReconstructionIndex];
    }

    LG_Log(
        LogSeverity::DBG,
        "[INIT_ProcessNewFrame] Selected reconstruction %llu with %zu map points\n",
        static_cast<unsigned long long>(BestReconstructionIndex),
        MostPoints);

    LG_Log(
        LogSeverity::DBG,
        "[INIT_ProcessNewFrame] Selected initialization frames %llu and %llu\n",
        static_cast<unsigned long long>(InitData.InitFrames[CandidateIDs[BestReconstructionIndex]].ID),
        static_cast<unsigned long long>(InitData.InitFrames.back().ID));

    return Reconstruction[BestReconstructionIndex];
}

void INIT_DestroyInitData(void)
{
    InitData.InitFrames.clear();
    InitData.FeatureTracks.clear();
    InitData.EnoughStationaryPointsForInit = false;
    InitNumFrames = 1;
}

typeGlobalMap INIT_ConstructInitialMap(typeInitReconstruction Reconstruction)
{
    assert(Reconstruction.Valid == true);
    assert(Reconstruction.MapPoints.size() > 0);
    assert(Reconstruction.NumPointsInFront > 0);

    const u64 FirstFrameID = Reconstruction.ChosenInitFrameID.first;
    const u64 SecondFrameID = Reconstruction.ChosenInitFrameID.second;
    LG_Log(LogSeverity::DBG, "FirstFrameID = %llu\n", FirstFrameID); 
    LG_Log(LogSeverity::DBG, "SecondFrameID= %llu\n", SecondFrameID); 

    typePantoVector<typePantoMapPoint> InitialMapPoints;
    InitialMapPoints.reserve(Reconstruction.MapPoints.size());

    typePantoVector<typePantoKeypointFrame> ImagePoints;
    ImagePoints.reserve(2);

    ImagePoints.push_back(INITPriv_GetKeyPointFrame(FirstFrameID));
    ImagePoints.push_back(INITPriv_GetKeyPointFrame(SecondFrameID));

    for(std::size_t i{}; i < Reconstruction.MapPoints.size(); i++)
    {
        const typeInitMapPoint& InitMapPoint = Reconstruction.MapPoints[i];

        typePantoVector<u64> ImagePointIDs(2, 0);
        ImagePointIDs[0] = InitMapPoint.InitImagePointID.first;
        ImagePointIDs[1] = InitMapPoint.InitImagePointID.second;

        typeDescriptor Descriptor = InitData.InitFrames[SecondFrameID].ImagePoints[ImagePointIDs[1]].Descriptor;

        typePantoVector<u64> KeyFrameIDs(2, 0);
        KeyFrameIDs[0] = 0;
        KeyFrameIDs[1] = 1;

        InitialMapPoints.push_back(
            {
                .Point = InitMapPoint.Point4D,
                .Descriptor = Descriptor,
                .KeyFrameIDs = KeyFrameIDs,
                .ImagePointIDs = ImagePointIDs,
                .ID = static_cast<u64>(i),
                .NumVisible = 1,
                .NumFound = 1,
                .CreationAge = KeyFrameIDs[1]
            });

        ImagePoints[0].ImagePoints[ImagePointIDs[0]].MapPointID = static_cast<u64>(i);
        ImagePoints[1].ImagePoints[ImagePointIDs[1]].MapPointID = static_cast<u64>(i);
    }

    Eigen::Matrix3d FirstR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d Firstt(0, 0, 0);
    typeCamera FirstCamera = CM_CreateCam(FirstR, Firstt, InitData.InitFrames[FirstFrameID].TimeStamp);

    Eigen::Matrix3d SecondR = Reconstruction.R;
    Eigen::Vector3d Secondt = Reconstruction.t;
    typeCamera SecondCamera = CM_CreateCam(SecondR, Secondt, InitData.InitFrames[SecondFrameID].TimeStamp);

    typePantoVector<typeKeyFrame> KeyFrames;
    KeyFrames.push_back(
        {
            .Points = ImagePoints[0],
            .BowVector = InitData.InitFrames[FirstFrameID].BoWVector,
            .FeatureVector = InitData.InitFrames[FirstFrameID].FeatureVector,
            .Pose = FirstCamera,
            .ID = 0,
            .ImagePath = InitData.InitFrames[FirstFrameID].ImagePath
        });

    KeyFrames.push_back(
        {
            .Points = ImagePoints[1],
            .BowVector = InitData.InitFrames[SecondFrameID].BoWVector,
            .FeatureVector = InitData.InitFrames[SecondFrameID].FeatureVector,
            .Pose = SecondCamera,
            .ID = 1,
            .ImagePath = InitData.InitFrames[SecondFrameID].ImagePath
        });

    return {.KeyFrames = KeyFrames, .MapPoints = InitialMapPoints, 0};
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

                        u32 Distance = PANTO_HammingDistance(ImagePointNew.Descriptor, ImagePointHistorical.Descriptor);

                        if(Distance < BestDistance)
                        {
                            SecondBestDistance = BestDistance;
                            BestDistance = Distance;
                            BestFeatureID = static_cast<u64>(FeatureIDHistorical);
                        }
                        else if(Distance < SecondBestDistance)
                        {
                            SecondBestDistance = Distance;
                        }
                    }
                    if(SecondBestDistance == PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1) continue;
                    if((static_cast<fp64>(BestDistance) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestDistance))
                            && (BestDistance < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW))
                    {
                        const typeInitImagePoint& TopCandidate = HistoricalFrame.ImagePoints[BestFeatureID];
                        const u64 FeatureTrackID = TopCandidate.FeatureTrackID;

                        if(FeatureTrackID == PANTO_ID_NOT_SET)
                        {
                                const u64 NewFeatureTrackID =
                                    static_cast<u64>(InitData.FeatureTracks.size());

                                std::vector<u64> NewTrack(
                                    InitData.InitFrames.size(),
                                    PANTO_ID_NOT_SET);

                                NewTrack[HistoricalFrame.ID] =
                                    TopCandidate.ID;

                                NewTrack[LatestFrame.ID] =
                                    ImagePointNew.ID;

                                typeFeatureTrack FeatureTrack =
                                {
                                    .FeatureTrack = NewTrack,
                                    .InlierCount = 0,
                                    .OutlierCount = 0
                                };

                                InitData.FeatureTracks.push_back(
                                    std::move(FeatureTrack));

                                ImagePointNew.FeatureTrackID =
                                    NewFeatureTrackID;

                                HistoricalFrame.ImagePoints[BestFeatureID].FeatureTrackID =
                                    NewFeatureTrackID;
                        }
                        else
                        {
                            typeFeatureTrack& FeatureTrack =
                                InitData.FeatureTracks[FeatureTrackID];

                            assert(FeatureTrack.FeatureTrack.size() ==
                                    InitData.InitFrames.size());

                            if(FeatureTrack.FeatureTrack[LatestFrame.ID] !=
                                    PANTO_ID_NOT_SET)
                            {
                                continue;
                            }

                            FeatureTrack.FeatureTrack[LatestFrame.ID] =
                                ImagePointNew.ID;

                            ImagePointNew.FeatureTrackID =
                                FeatureTrackID;
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

        if(LatestFramePoints.size() < PANTO_FUNDAMENTAL_MIN_POINTS)
        {
            LG_Log( LogSeverity::DBG, "[INITPriv_STRANSAC] Frame %llu has only %zu common tracks, skipping\n",
                static_cast<unsigned long long>(HistoricalFrame.ID),
                LatestFramePoints.size());

            continue;
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

    LG_Log(LogSeverity::DBG,"[INITPriv_ScoredFAndHEstimation] Fundamental score: %lf, Homography score: %lf\n", Fundamental->MaxScore, Homography->MaxScore);

    const fp64 FundamentalScore = Fundamental->MaxScore;
    const fp64 HomographyScore = Homography->MaxScore;

    const fp64 TotalScore = FundamentalScore + HomographyScore;

    if(TotalScore <= std::numeric_limits<fp64>::epsilon())
    {
        return Fundamental;
    }

    const fp64 HomographyRatio = HomographyScore / TotalScore;

    LG_Log( LogSeverity::DBG,
            "[INITPriv_ScoredFAndHEstimation] " "F score = %.3f, H score = %.3f, H ratio = %.3f\n",
            FundamentalScore, HomographyScore, HomographyRatio);

    if(HomographyRatio > 0.45)
    {
        return Homography;
    }

    return Fundamental;
}

std::pair<std::unique_ptr<ImageToImageMapping>, std::unique_ptr<ImageToImageMapping>> INITPriv_FAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical)
{
    std::unique_ptr<FundamentalMatrixMapping> Fundamental = std::make_unique<FundamentalMatrixMapping>();
    std::unique_ptr<HomographyMapping> Homography = std::make_unique<HomographyMapping>();

    const u64 Seed = INITPriv_RandomSeed();

    std::thread HomographyThread(&HomographyMapping::Estimate, Homography.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical), Seed);
    std::thread FundamentalThread(&FundamentalMatrixMapping::Estimate, Fundamental.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical), Seed);

    HomographyThread.join();
    FundamentalThread.join();

    LG_Log(LogSeverity::DBG,"[INITPriv_ScoredFAndHEstimation] Fundamental score: %lf, Homography score: %lf\n", Fundamental->MaxScore, Homography->MaxScore);

    return {std::move(Fundamental), std::move(Homography)};
}

u64 INITPriv_RandomSeed(void)
{
    std::random_device RandomDevice;

    u64 High = static_cast<u64>(RandomDevice());
    u64 Low  = static_cast<u64>(RandomDevice());

    return (High << 32) | Low;
}

std::vector<u64> INITPriv_GetCandidateFrameIDs(const std::vector<u64>& StationaryTrackIDs)
{
    std::vector<u64> Covisibility( InitData.InitFrames.size(), 0);

    const u64 LatestFrameID = InitData.InitFrames.back().ID;

    for(const u64 TrackID : StationaryTrackIDs)
    {
        const typeFeatureTrack& FeatureTrack = InitData.FeatureTracks[TrackID];

        for(u64 i = 0; i < LatestFrameID; ++i)
        {
            if(FeatureTrack.FeatureTrack[i] != PANTO_ID_NOT_SET)
            {
                ++Covisibility[i];
            }
        }
    }

    std::vector<u64> CandidateFrameIDs;
    CandidateFrameIDs.reserve(LatestFrameID);

    for(u64 i = 0; i < LatestFrameID; ++i)
    {
        if(Covisibility[i] >= PANTO_FUNDAMENTAL_MIN_POINTS)
        {
            CandidateFrameIDs.push_back(i);
        }
    }

    std::sort( CandidateFrameIDs.begin(), CandidateFrameIDs.end(),
        [&Covisibility](const u64 A, const u64 B)
        {
            return Covisibility[A] > Covisibility[B];
        });

    const std::size_t MaxCandidates =
        PANTO_NUM_THREADS_MAX * PANTO_INIT_CANDIDATE_BATCHES;
    const std::size_t NumCandidates =
        std::min<std::size_t>(MaxCandidates, CandidateFrameIDs.size());

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

    std::pair<std::unique_ptr<ImageToImageMapping>, std::unique_ptr<ImageToImageMapping>> Mappings = INITPriv_FAndHEstimation(HistoricalPoints, NewPoints);

    const typeCameraIntrinsics* Intrinsics = CM_GetIntrinsics();

    const Eigen::Matrix3d& K = Intrinsics->K;

    const fp64 FundamentalScore = Mappings.first->MaxScore;
    const fp64 HomographyScore = Mappings.second->MaxScore;

    const fp64 TotalScore = FundamentalScore + HomographyScore;

    std::unique_ptr<ImageToImageMapping> Mapping{};
    std::unique_ptr<ImageToImageMapping> OtherMapping{};

    if(TotalScore <= std::numeric_limits<fp64>::epsilon())
    {
        Mapping = std::move(Mappings.first);
        OtherMapping = std::move(Mappings.second);
    }
    else
    {

        const fp64 HomographyRatio = HomographyScore / TotalScore;

        LG_Log( LogSeverity::DBG,
                "[INITPriv_ScoredFAndHEstimation] " "F score = %.3f, H score = %.3f, H ratio = %.3f\n",
                FundamentalScore, HomographyScore, HomographyRatio);

        if(HomographyRatio > 0.45)
        {
            Mapping = std::move(Mappings.second);
            OtherMapping = std::move(Mappings.first);
        }
        else
        {
            Mapping = std::move(Mappings.first);
            OtherMapping = std::move(Mappings.second);
        }
    }

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

    if(!Reconstruction.Valid)
    {
        Reconstruction = OtherMapping->Reconstruct(
            HistoricalPoints,
            NewPoints,
            ImagePointIDs,
            FrameIDs,
            K);
    }

    return Reconstruction;
}

void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp, const std::string& ImagePath)
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

    DBoW3::Vocabulary* Vocabulary = DBOW3_GetVocabulary();
    Vocabulary->transform(DescriptorVector, InitFrame.BoWVector, InitFrame.FeatureVector, Levels);

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

    InitFrame.ImagePath = ImagePath;
    InitFrame.ID = static_cast<u64>(InitFrames.size());
    InitFrame.TimeStamp = TimeStamp;
    InitFrames.push_back(InitFrame);

    for(typeFeatureTrack& FeatureTrack : InitData.FeatureTracks)
    {
        FeatureTrack.FeatureTrack.push_back(
                PANTO_ID_NOT_SET);
    }
}

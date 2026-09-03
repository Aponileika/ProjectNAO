#include "KEY_KeyFrame.hpp"
#include "KEY_KeyFramePriv.hpp"

// Since getkeyframe can be reached before setaskeyframe is called this has to be a deque (or just a que really)
static std::queue<cv::Mat> CurrentDescriptors{};

struct typeKeyFrameTimingStatistics
{
    u64 Count = 0;
    fp64 Sum = 0.0;
    fp64 SumSquared = 0.0;
};

static typeKeyFrameTimingStatistics GetKeyFrameTotalTiming{};
static typeKeyFrameTimingStatistics GetFrameTiming{};
static typeKeyFrameTimingStatistics GetDescriptorsTiming{};
static typeKeyFrameTimingStatistics CreateImagePointsTiming{};

struct typeIsKeyFrameStatistics
{
    u64 Evaluations = 0;
    u64 TrueDecisions = 0;
    u64 BootstrapTrueDecisions = 0;
    u64 FuzzyEvaluations = 0;
    u64 FuzzyTrueDecisions = 0;
    u64 MaxRuleTriggered = 0;
    u64 SpatialRuleTriggered = 0;
    u64 MaxRuleOnly = 0;
    u64 SpatialRuleOnly = 0;
    u64 BothRules = 0;
    u64 MaxVelocityTriggered = 0;
    u64 MaxDistanceTriggered = 0;
    u64 MaxTrackingTriggered = 0;
    u64 SpatialVelocityTrackingTriggered = 0;
    u64 SpatialDistanceTrackingTriggered = 0;
};

static typeIsKeyFrameStatistics IsKeyFrameStatistics{};

static void KEYPriv_AddTimingSample(typeKeyFrameTimingStatistics& Statistics, const fp64 Time)
{
    Statistics.Count++;
    Statistics.Sum += Time;
    Statistics.SumSquared += Time * Time;
}

static void KEYPriv_LogTimingStatistics(const char* Name, const typeKeyFrameTimingStatistics& Statistics)
{
    if(Statistics.Count == 0)
    {
        LG_Log(LogSeverity::DATA, "[SLAMTimingSummary] %s: no samples\n", Name);
        return;
    }

    const fp64 Mean = Statistics.Sum / static_cast<fp64>(Statistics.Count);
    const fp64 Variance = std::max<fp64>(
            0.0,
            Statistics.SumSquared / static_cast<fp64>(Statistics.Count) - Mean * Mean);
    const fp64 StandardDeviation = std::sqrt(Variance);

    LG_Log(LogSeverity::DATA,
            "[SLAMTimingSummary] %s: mean = %.6f s, std dev = %.6f s, samples = %llu\n",
            Name,
            Mean,
            StandardDeviation,
            static_cast<unsigned long long>(Statistics.Count));
}

static typeKeyFrameBootStrapData BootStrapData = 
{
    .VelocityChange{},
    .LocalMapTrackingRatio{},
    .AcumulatedDistanceTravelled{},
    .NumFrames{},
    .BootStrapDataSolved = false
};

typeFuzzyKeyFrameInference FuzzyInference = 
{
    .VelocityParamameters{},
    .TrackingRatioParameters{},
    .AccumulatedDistanceParameters{},
    .MaxRuleThreshold = NAN,
    .SpatialTrackingThreshold = NAN
};

typeKeyFrame KEY_CreateKeyFrame(const typeNavigationState& NavState, const typePantoFrame& Frame)
{
    typeKeyFrame KeyFrame{};

    DBoW3::Vocabulary* Vocab = DBOW3_GetVocabulary();
    const typePose BodyToCamera = CM_GetBodyToSensor(CM_GetIntrinsics());
    DescRet Desc = EP_GetDescriptors(Frame.Frame);
    std::vector<cv::Mat> DescriptorVector;
    DescriptorVector.reserve(Desc.Descriptors.rows);
    for(i32 i{}; i < Desc.Descriptors.rows; i++)
    {
        DescriptorVector.push_back(Desc.Descriptors.row(i));
    }

    const i32 Levels = PANTO_DBOW_LEVELSUP;

    Vocab->transform(DescriptorVector, KeyFrame.BowVector, KeyFrame.FeatureVector, Levels);

    KeyFrame.ID = 0;
    KeyFrame.ImagePath = Frame.Path;
    KeyFrame.NavigationState = NavState;

    const Eigen::Matrix3d& CameraR = BodyToCamera.R * NavState.Rwb.transpose();
    const Eigen::Vector3d& Camerat = BodyToCamera.R * NavState.t + BodyToCamera.t;

    KeyFrame.Camera = CM_CreateCam(CameraR, Camerat, Frame.TimeStamp);

    return KeyFrame;
}

typeKeyFrame KEY_GetThirdKeyFrame(typeKeyFrame& LastKeyFrame, typePantoVector<typePantoMapPoint>& GlobalMapPoints)
{
    typePantoFrame Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);

    typePantoVector<cv::Mat> DescriptorVector;

    DescriptorVector.reserve(Descriptors.Descriptors.rows);

    for(i32 i{}; i < Descriptors.Descriptors.rows; i++)
    {
        DescriptorVector.push_back(Descriptors.Descriptors.row(i));
    }

    u64 LevelsUp = PANTO_DBOW_LEVELSUP;
    DBoW3::Vocabulary* Vocabulary = DBOW3_GetVocabulary();

    DBoW3::BowVector NewBowVector;
    DBoW3::FeatureVector NewFeatureVector;

    Vocabulary->transform(DescriptorVector, NewBowVector, NewFeatureVector, LevelsUp);

    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePointsNoMatch(Descriptors.Points, Descriptors.Descriptors);

    typeCamera PredictedPose = LastKeyFrame.Camera;
    PredictedPose.TimeStamp = Frame.TimeStamp;

    typeKeyFrame KeyFrame = 
    {
        .Points = ImagePoints,
        .BowVector = NewBowVector,
        .FeatureVector = NewFeatureVector,
        .Camera = PredictedPose,
        .ID = 2,
        .ImagePath = Frame.Path
    };

    const DBoW3::FeatureVector& FeatureVector1 = KeyFrame.FeatureVector;
    const DBoW3::FeatureVector& FeatureVector2 = LastKeyFrame.FeatureVector;

    auto FeatureIterator1 = FeatureVector1.begin();
    auto FeatureIterator2 = FeatureVector2.begin();

    typePantoVector<typePantoImagePoint>& AllImagePoints1 = KeyFrame.Points.ImagePoints;
    typePantoVector<typePantoImagePoint>& AllImagePoints2 = LastKeyFrame.Points.ImagePoints;

    u64 NumMatches = 0;

    std::unordered_set<u64> VisibleMapPointIDs;
    std::unordered_set<u64> MatchedMapPointIDs;

    while(FeatureIterator1 != FeatureVector1.end() && FeatureIterator2 != FeatureVector2.end())
    {
        if(FeatureIterator1->first == FeatureIterator2->first)
        {
            //Feature vector match
            const std::vector<u32>& FeatureIDs1 = FeatureIterator1->second;
            const std::vector<u32>& FeatureIDs2 = FeatureIterator2->second;

            for(const u32& FeatureID1 : FeatureIDs1)
            {
                typePantoImagePoint& ImagePoint1 = AllImagePoints1[FeatureID1];

                u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                u32 SecondBestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                u64 BestFeatureID = PANTO_ID_NOT_SET;

                if(ImagePoint1.MapPointID != PANTO_ID_NOT_SET)
                {
                    continue;
                }

                for(const u32& FeatureID2 : FeatureIDs2)
                {
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[FeatureID2];

                    if(ImagePoint2.MapPointID == PANTO_ID_NOT_SET)
                    {
                        continue;
                    }

                    typePantoMapPoint& MapPoint = GlobalMapPoints[ImagePoint2.MapPointID];

                    // Eigen::Vector2d ProjectedPoint{};

                    // if(!PROJ_Project(MapPoint.Point, ProjectedPoint, KeyFrame.Camera))
                    // {
                    //     continue;
                    // }

                    if(VisibleMapPointIDs.insert(MapPoint.ID).second)
                    {
                        MapPoint.NumVisible++;
                    }
                    
                    // const fp64 ProjectionError =
                    //     (ProjectedPoint - ImagePoint1.Point).norm();

                    // if(ProjectionError > PANTO_MAPPOINT_MATCH_SEARCH_RADIUS)
                    // {
                    //     continue;
                    // }

                    const u32 Distance = PANTO_HammingDistance(ImagePoint1.Descriptor, ImagePoint2.Descriptor);

                    if(Distance < BestDistance)
                    {
                        SecondBestDistance = BestDistance;
                        BestDistance = Distance;
                        BestFeatureID = static_cast<u64>(FeatureID2);
                    }
                    else if(Distance < SecondBestDistance)
                    {
                        SecondBestDistance = Distance;
                    }
                }

                if(BestFeatureID != PANTO_ID_NOT_SET)
                {
                    const typePantoImagePoint& BestMatch = LastKeyFrame.Points.ImagePoints[BestFeatureID];

                    if((BestDistance < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW) &&
                        (static_cast<fp64>(BestDistance) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestDistance))
                        && MatchedMapPointIDs.insert(BestMatch.MapPointID).second)
                    {
                        ImagePoint1.MapPointID = BestMatch.MapPointID;

                        for(const u64 ExistingKeyFrameID : GlobalMapPoints[BestMatch.MapPointID].KeyFrameIDs)
                        {
                            assert(ExistingKeyFrameID != 2);
                        }

                        GlobalMapPoints[BestMatch.MapPointID].KeyFrameIDs.push_back(2);
                        GlobalMapPoints[BestMatch.MapPointID].ImagePointIDs.push_back(ImagePoint1.ID);
                        GlobalMapPoints[BestMatch.MapPointID].NumFound++;
                        NumMatches++;
                    }
                }
            }

            ++FeatureIterator1;
            ++FeatureIterator2;
        }
        else if(FeatureIterator1->first < FeatureIterator2->first)
        {
            FeatureIterator1 = FeatureVector1.lower_bound(FeatureIterator2->first);
        }
        else
        {
            FeatureIterator2 = FeatureVector2.lower_bound(FeatureIterator1->first);
        }
    }

    LG_Log(LogSeverity::DBG, "[KEY_GetThirdKeyFrame] Matched %llu map points\n", static_cast<unsigned long long>(NumMatches));
    return KeyFrame;
}

typeKeyFrame KEY_GetKeyFrame(typeCamera& PredictedPose, std::vector<typePantoMapPoint>& LastFrameMapPoints,
        typePantoVector<typePantoMapPoint>& GlobalMapPoints)
{
    const PantoClock::time_point GetKeyFrameStartTime = PantoClock::now();

    LG_Log(LogSeverity::DBG, "[KEY_GetKeyFrame] Predicted q = (%f, %f, %f, %f), t = (%f, %f, %f)\n",
        PredictedPose.Pose.Quaternion.w(),
        PredictedPose.Pose.Quaternion.x(),
        PredictedPose.Pose.Quaternion.y(),
        PredictedPose.Pose.Quaternion.z(),
        PredictedPose.Pose.tParametrization[0],
        PredictedPose.Pose.tParametrization[1],
        PredictedPose.Pose.tParametrization[2]);

    const PantoClock::time_point GetFrameStartTime = PantoClock::now();
    typePantoFrame Frame = FR_GetFrame();
    const fp64 GetFrameTime =
        std::chrono::duration<fp64>(PantoClock::now() - GetFrameStartTime).count();

    KEYPriv_AddTimingSample(GetFrameTiming, GetFrameTime);

    if(Frame.TimeStamp < 0.0f)
    {
        PredictedPose.TimeStamp = -1.0f;

        const fp64 GetKeyFrameTotalTime =
            std::chrono::duration<fp64>(PantoClock::now() - GetKeyFrameStartTime).count();

        KEYPriv_AddTimingSample(GetKeyFrameTotalTiming, GetKeyFrameTotalTime);

        LG_Log(LogSeverity::DBG,
                "[KEY_GetKeyFrameTiming] total = %.6f s, FR_GetFrame = %.6f s, frame invalid\n",
                GetKeyFrameTotalTime,
                GetFrameTime);

        return
        {
            .Points = typePantoKeypointFrame{},
            .BowVector = {},
            .FeatureVector = {},
            .Camera = PredictedPose,
            .ID = PANTO_ID_NOT_SET,
            .ImagePath = "" 
        };
    }
    PredictedPose.TimeStamp = Frame.TimeStamp;

    const PantoClock::time_point GetDescriptorsStartTime = PantoClock::now();
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);
    const fp64 GetDescriptorsTime =
        std::chrono::duration<fp64>(PantoClock::now() - GetDescriptorsStartTime).count();

    KEYPriv_AddTimingSample(GetDescriptorsTiming, GetDescriptorsTime);

    CurrentDescriptors.push(Descriptors.Descriptors);

    const PantoClock::time_point CreateImagePointsStartTime = PantoClock::now();
    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePoints(Descriptors.Points, Descriptors.Descriptors, LastFrameMapPoints, PredictedPose,
            GlobalMapPoints);
    const fp64 CreateImagePointsTime =
        std::chrono::duration<fp64>(PantoClock::now() - CreateImagePointsStartTime).count();

    KEYPriv_AddTimingSample(CreateImagePointsTiming, CreateImagePointsTime);

    typeKeyFrame KeyFrame = 
    {
        .Points = ImagePoints,
        .BowVector = {},
        .FeatureVector = {},
        .Camera = PredictedPose,
        .ID = PANTO_ID_NOT_SET,
        .ImagePath = Frame.Path
    };

    const fp64 GetKeyFrameTotalTime =
        std::chrono::duration<fp64>(PantoClock::now() - GetKeyFrameStartTime).count();

    KEYPriv_AddTimingSample(GetKeyFrameTotalTiming, GetKeyFrameTotalTime);

    LG_Log(LogSeverity::DBG,
            "[KEY_GetKeyFrameTiming] total = %.6f s, FR_GetFrame = %.6f s, EP_GetDescriptors = %.6f s, PT_CreatePantoImagePoints = %.6f s\n",
            GetKeyFrameTotalTime,
            GetFrameTime,
            GetDescriptorsTime,
            CreateImagePointsTime);

    return KeyFrame;
}

void KEY_LogGetKeyFrameTimingStatistics(void)
{
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/internal total", GetKeyFrameTotalTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/FR_GetFrame", GetFrameTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/EP_GetDescriptors", GetDescriptorsTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/PT_CreatePantoImagePoints", CreateImagePointsTiming);
}

void KEY_LogIsKeyFrameStatistics(void)
{
    const typeIsKeyFrameStatistics& Statistics = IsKeyFrameStatistics;

    LG_Log(LogSeverity::DATA,
            "[SLAMKeyFrameDecisionSummary] Evaluations = %llu, true = %llu, false = %llu\n",
            static_cast<unsigned long long>(Statistics.Evaluations),
            static_cast<unsigned long long>(Statistics.TrueDecisions),
            static_cast<unsigned long long>(Statistics.Evaluations - Statistics.TrueDecisions));
    LG_Log(LogSeverity::DATA,
            "[SLAMKeyFrameDecisionSummary] True decision source: bootstrap = %llu, fuzzy rules = %llu\n",
            static_cast<unsigned long long>(Statistics.BootstrapTrueDecisions),
            static_cast<unsigned long long>(Statistics.FuzzyTrueDecisions));
    LG_Log(LogSeverity::DATA,
            "[SLAMKeyFrameDecisionSummary] Fuzzy evaluations = %llu; max only = %llu, spatial only = %llu, both = %llu\n",
            static_cast<unsigned long long>(Statistics.FuzzyEvaluations),
            static_cast<unsigned long long>(Statistics.MaxRuleOnly),
            static_cast<unsigned long long>(Statistics.SpatialRuleOnly),
            static_cast<unsigned long long>(Statistics.BothRules));
    LG_Log(LogSeverity::DATA,
            "[SLAMKeyFrameDecisionSummary] Max rule triggered = %llu; parameters: velocity = %llu, distance = %llu, tracking = %llu\n",
            static_cast<unsigned long long>(Statistics.MaxRuleTriggered),
            static_cast<unsigned long long>(Statistics.MaxVelocityTriggered),
            static_cast<unsigned long long>(Statistics.MaxDistanceTriggered),
            static_cast<unsigned long long>(Statistics.MaxTrackingTriggered));
    LG_Log(LogSeverity::DATA,
            "[SLAMKeyFrameDecisionSummary] Spatial rule triggered = %llu; branches: velocity+tracking = %llu, distance+tracking = %llu\n",
            static_cast<unsigned long long>(Statistics.SpatialRuleTriggered),
            static_cast<unsigned long long>(Statistics.SpatialVelocityTrackingTriggered),
            static_cast<unsigned long long>(Statistics.SpatialDistanceTrackingTriggered));
}

void KEY_Reset(void)
{
    CurrentDescriptors = {};
    IsKeyFrameStatistics = {};

    BootStrapData =
    {
        .VelocityChange{},
        .LocalMapTrackingRatio{},
        .AcumulatedDistanceTravelled{},
        .NumFrames{},
        .BootStrapDataSolved = false
    };

    FuzzyInference =
    {
        .VelocityParamameters{},
        .TrackingRatioParameters{},
        .AccumulatedDistanceParameters{},
        .MaxRuleThreshold = NAN,
        .SpatialTrackingThreshold = NAN
    };
}

bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information)
{
    IsKeyFrameStatistics.Evaluations++;

    LG_Log( LogSeverity::DBG,
        "[KEY_IsKeyFrame] Incoming information: VelocityChange = %f, LocalMapTrackingRatio = %f, AcumulatedDistanceTravelled = %f\n",
        Information.VelocityChange,
        Information.LocalMapTrackingRatio,
        Information.AcumulatedDistanceTravelled);

    LG_Log( LogSeverity::DBG,
        "[KEY_IsKeyFrame] Bootstrap before update: NumFrames = %llu, VelocityChange = %f, LocalMapTrackingRatio = %f, AcumulatedDistanceTravelled = %f\n",
        static_cast<unsigned long long>(BootStrapData.NumFrames),
        BootStrapData.VelocityChange,
        BootStrapData.LocalMapTrackingRatio,
        BootStrapData.AcumulatedDistanceTravelled);

    if(BootStrapData.NumFrames < PANTO_NUM_BOOTSTRAP_FRAMES)
    {
        ++BootStrapData.NumFrames;
        BootStrapData.AcumulatedDistanceTravelled += Information.AcumulatedDistanceTravelled;
        BootStrapData.LocalMapTrackingRatio       += Information.LocalMapTrackingRatio;
        BootStrapData.VelocityChange              += Information.VelocityChange;

        LG_Log( LogSeverity::DBG,
            "[KEY_IsKeyFrame] Bootstrap after update: NumFrames = %llu, VelocityChange = %f, LocalMapTrackingRatio = %f, AcumulatedDistanceTravelled = %f\n",
            static_cast<unsigned long long>(BootStrapData.NumFrames),
            BootStrapData.VelocityChange,
            BootStrapData.LocalMapTrackingRatio,
            BootStrapData.AcumulatedDistanceTravelled);

        IsKeyFrameStatistics.TrueDecisions++;
        IsKeyFrameStatistics.BootstrapTrueDecisions++;

        return true;
    }
    if(!BootStrapData.BootStrapDataSolved)
    {
        KEYPriv_SolveBootStrapData();
    }
    const bool Decision = KEYPriv_IsKeyFrame(Information);

    if(Decision)
    {
        IsKeyFrameStatistics.TrueDecisions++;
        return true;
    }

    return false;
}

void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, typePantoVector<typePantoMapPoint>& GlobalMapPoints, 
        const typePantoVector<typeKeyFrame>& GlobalKeyFrames, const DBoW3::Vocabulary* Vocabulary)
{
    const u64 ID = KeyFrame.ID;
    LG_Log(LogSeverity::DBG, "[KEY_SetAsKeyFrame] ID = %llu\n", ID);
    const cv::Mat& Descriptors = CurrentDescriptors.front();
    std::vector<cv::Mat> DescriptorVector;
    DescriptorVector.reserve(Descriptors.rows);
    for(i32 i{}; i < Descriptors.rows; i++)
    {
        DescriptorVector.push_back(Descriptors.row(i));
    }

    const i32 Levels = PANTO_DBOW_LEVELSUP;

    for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        const u64 ImagePointID = ImagePoint.ID;
        typePantoMapPoint& MapPoint = GlobalMapPoints[MapPointID];
        for(const u64 ExistingKeyFrameID : MapPoint.KeyFrameIDs)
        {
            if(ExistingKeyFrameID == ID)
            {
                LG_Log( LogSeverity::ERROR,
                        "[KEY_SetAsKeyFrame] MP %llu already contains KF %llu, current ImagePoint = %llu\n",
                        MapPointID, ID, ImagePointID);

                assert(false);
            }
        }

        MapPoint.ImagePointIDs.push_back(ImagePointID);
        MapPoint.KeyFrameIDs.push_back(ID);

        std::vector<typeDescriptor> Descriptors;
        Descriptors.reserve(MapPoint.KeyFrameIDs.size());

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            if(MapPoint.ImagePointIDs.contains(i) && MapPoint.KeyFrameIDs.contains(i))
            {
                const u64 ImagePointID = MapPoint.ImagePointIDs[i];
                const u64 KeyFrameID   = MapPoint.KeyFrameIDs[i];

                Descriptors.push_back(GlobalKeyFrames[KeyFrameID]. Points.ImagePoints[ImagePointID]. Descriptor);
            }
        }

        if(Descriptors.size() == 1)
        {
            MapPoint.Descriptor = Descriptors[0];
            continue;
        }

        u64 BestDescriptorID = 0;
        fp64 BestMedianDistance = std::numeric_limits<fp64>::max();

        for(std::size_t i{}; i < Descriptors.size(); i++)
        {
            std::vector<i32> Distances;
            Distances.reserve(Descriptors.size() - 1);

            for(std::size_t j{}; j < Descriptors.size(); j++)
            {
                if(i == j)
                {
                    continue;
                }

                Distances.push_back(PANTO_HammingDistance( Descriptors[i],
                                Descriptors[j]));
            }

            std::sort(Distances.begin(), Distances.end());

            const fp64 MedianDistance = Distances[Distances.size() / 2];

            if(MedianDistance < BestMedianDistance)
            {
                BestMedianDistance = MedianDistance;
                BestDescriptorID = i;
            }
        }

        MapPoint.Descriptor = Descriptors[BestDescriptorID];
    }

    Vocabulary->transform(DescriptorVector, KeyFrame.BowVector, KeyFrame.FeatureVector, Levels);
    CurrentDescriptors.pop();
}

std::vector<u64> KEY_InsertNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, typePantoVector<typePantoMapPoint>& GlobalMapPoints,
        const u64 MapAge)
{
    std::vector<u64> Indexes;

    std::size_t NumSharedNodes = 0;
    std::size_t NumImagePoint1Candidates = 0;
    std::size_t NumImagePoint1Invalid = 0;
    std::size_t NumImagePoint1Associated = 0;
    std::size_t NumDescriptorComparisons = 0;
    std::size_t NumImagePoint2Invalid = 0;
    std::size_t NumImagePoint2AssociatedSkips = 0;
    std::size_t NumNotTopTwoRejected = 0;
    std::size_t NumEpipolarAccepted = 0;
    std::size_t NumEpipolarRejected = 0;
    std::size_t NumEpipolarNonFiniteRejected = 0;
    std::size_t NumEpipolarRejectedThresholdToThresholdPlusTwo = 0;
    std::size_t NumEpipolarRejectedThresholdPlusTwoToTen = 0;
    std::size_t NumEpipolarRejectedTenToTwenty = 0;
    std::size_t NumEpipolarRejectedTwentyPlus = 0;
    fp64 SumEpipolarDistanceAccepted = 0.0;
    fp64 SquaredSumEpipolarDistanceAccepted = 0.0;
    fp64 SumEpipolarDistanceRejected = 0.0;
    fp64 SquaredSumEpipolarDistanceRejected = 0.0;
    std::size_t NumParallaxRejected = 0;
    std::size_t NumNoBestMatchRejected = 0;
    std::size_t NumHammingRejected = 0;
    std::size_t NumBestHammingDistanceSamples = 0;
    fp64 SumBestHammingDistance = 0.0;
    fp64 SquaredSumBestHammingDistance = 0.0;
    std::size_t NumSecondBestHammingDistanceSamples = 0;
    fp64 SumSecondBestHammingDistance = 0.0;
    fp64 SquaredSumSecondBestHammingDistance = 0.0;
    std::size_t NumRatioRejected = 0;
    std::size_t NumNonFiniteRejected = 0;
    std::size_t NumProjectionRejected = 0;
    std::size_t NumProjectionNonFiniteRejected = 0;
    std::size_t NumDepth1OnlyRejected = 0;
    std::size_t NumDepth2OnlyRejected = 0;
    std::size_t NumBothDepthRejected = 0;
    std::size_t NumRejectedDepthSamples = 0;
    fp64 SumRejectedDepth1 = 0.0;
    fp64 SquaredSumRejectedDepth1 = 0.0;
    fp64 SumRejectedDepth2 = 0.0;
    fp64 SquaredSumRejectedDepth2 = 0.0;
    std::size_t NumReprojectionRejected = 0;
    fp64 SumRejectedReprojectionPixelError = 0.0;
    fp64 SquaredSumRejectedReprojectionPixelError = 0.0;
    std::size_t NumReprojectionRejectedThresholdToThresholdPlusTwo = 0;
    std::size_t NumReprojectionRejectedThresholdPlusTwoToTen = 0;
    std::size_t NumReprojectionRejectedTenToTwenty = 0;
    std::size_t NumReprojectionRejectedTwentyPlus = 0;
    std::size_t NumCheiralityRejected = 0;

    const Eigen::Matrix3d F21  = EP_GetFundamentalMatrix21(KeyFrame1.Camera.Pose, KeyFrame2.Camera.Pose);
    const Eigen::Matrix3d F12 = F21.transpose();

    typePantoVector<typePantoImagePoint>& AllImagePoints1 = KeyFrame1.Points.ImagePoints;
    typePantoVector<typePantoImagePoint>& AllImagePoints2 = KeyFrame2.Points.ImagePoints;

    const DBoW3::FeatureVector& FeatureVector1 = KeyFrame1.FeatureVector;
    const DBoW3::FeatureVector& FeatureVector2 = KeyFrame2.FeatureVector;

    auto FeatureIterator1 = FeatureVector1.begin();
    auto FeatureIterator2 = FeatureVector2.begin();

    std::pair<u64, u64> KeyFrameIDs(KeyFrame1.ID, KeyFrame2.ID);

    const typeCamera& Camera1 = KeyFrame1.Camera;
    const typeCamera& Camera2 = KeyFrame2.Camera;

    const Eigen::Matrix3d K = CM_GetIntrinsics()->K;

    Eigen::Matrix<fp64, 3, 4> Rt1 = CM_GetRt(KeyFrame1.Camera);
    Eigen::Matrix<fp64, 3, 4> Rt2 = CM_GetRt(KeyFrame2.Camera);

    const Eigen::Matrix<fp64, 3, 4> P1 = K * Rt1;

    const Eigen::Matrix<fp64, 3, 4> P2 = K * Rt2;

    const fp64 fx = K(0, 0);
    const fp64 fy = K(1, 1);

    const fp64 cx = K(0, 2);
    const fp64 cy = K(1, 2);
    
    while(FeatureIterator1 != FeatureVector1.end() && FeatureIterator2 != FeatureVector2.end())
    {
        if(FeatureIterator1->first == FeatureIterator2->first)
        {
            NumSharedNodes++;

            //Feature vector match
            const std::vector<u32>& FeatureIDs1 = FeatureIterator1->second;
            const std::vector<u32>& FeatureIDs2 = FeatureIterator2->second;

            for(const u32& FeatureID1 : FeatureIDs1)
            {
                if(!AllImagePoints1.contains(static_cast<u64>(FeatureID1)))
                {
                    NumImagePoint1Invalid++;
                    continue;
                }
                typePantoImagePoint& ImagePoint1 = AllImagePoints1[FeatureID1];

                u32 BestDistance = std::numeric_limits<u32>::max();

                u32 SecondBestDistance = std::numeric_limits<u32>::max();
                u64 BestFeatureID = PANTO_ID_NOT_SET;

                if(ImagePoint1.MapPointID != PANTO_ID_NOT_SET)
                {
                    NumImagePoint1Associated++;
                    continue;
                }

                NumImagePoint1Candidates++;

                const Eigen::Vector3d Ray1Camera =
                {
                    (ImagePoint1.Point.x() - cx) / fx,
                    (ImagePoint1.Point.y() - cy) / fy,
                    1.0
                };


                const Eigen::Vector3d Ray1World = Camera1.Pose.R.transpose() * Ray1Camera;

                for(const u32& FeatureID2 : FeatureIDs2)
                {
                    if(!AllImagePoints2.contains(static_cast<u64>(FeatureID2)))
                    {
                        NumImagePoint2Invalid++;
                        continue;
                    }
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[FeatureID2];

                    if(ImagePoint2.MapPointID != PANTO_ID_NOT_SET)
                    {
                        NumImagePoint2AssociatedSkips++;
                        continue;
                    }

                    NumDescriptorComparisons++;

                    const u32 Distance = PANTO_HammingDistance(ImagePoint1.Descriptor, ImagePoint2.Descriptor);

                    if(Distance >= SecondBestDistance)
                    {
                        NumNotTopTwoRejected++;
                        continue;
                    }

                    const fp64 MeanEpipolarDistance =
                        EP_CheckEpipolarConstraint(ImagePoint1.Point, ImagePoint2.Point, F21, F12);

                    if(MeanEpipolarDistance >= PANTO_EPIPOLARTRESHOLD)
                    {
                        NumEpipolarRejected++;

                        if(std::isfinite(MeanEpipolarDistance))
                        {
                            SumEpipolarDistanceRejected += MeanEpipolarDistance;
                            SquaredSumEpipolarDistanceRejected +=
                                MeanEpipolarDistance * MeanEpipolarDistance;

                            if(MeanEpipolarDistance < PANTO_EPIPOLARTRESHOLD + 2.0)
                            {
                                NumEpipolarRejectedThresholdToThresholdPlusTwo++;
                            }
                            else if(MeanEpipolarDistance < 10.0)
                            {
                                NumEpipolarRejectedThresholdPlusTwoToTen++;
                            }
                            else if(MeanEpipolarDistance < 20.0)
                            {
                                NumEpipolarRejectedTenToTwenty++;
                            }
                            else
                            {
                                NumEpipolarRejectedTwentyPlus++;
                            }
                        }
                        else
                        {
                            NumEpipolarNonFiniteRejected++;
                        }

                        continue;
                    }

                    NumEpipolarAccepted++;
                    SumEpipolarDistanceAccepted += MeanEpipolarDistance;
                    SquaredSumEpipolarDistanceAccepted +=
                        MeanEpipolarDistance * MeanEpipolarDistance;


                    const Eigen::Vector3d Ray2Camera =
                    {
                        (ImagePoint2.Point.x() - cx) / fx,
                        (ImagePoint2.Point.y() - cy) / fy,
                        1.0
                    };


                    const Eigen::Vector3d Ray2World = Camera2.Pose.R.transpose() * Ray2Camera;

                    const fp64 CosParallax = Ray1World.normalized().dot(Ray2World.normalized());

                    if(CosParallax <= 0 ||
                        CosParallax > PANTO_MAXIMUMCOSPARALLAX)
                    {
                        NumParallaxRejected++;
                        continue;
                    }

                    if(Distance < BestDistance)
                    {
                        SecondBestDistance = BestDistance;
                        BestDistance = Distance;
                        BestFeatureID = static_cast<u64>(FeatureID2);
                    }
                    else
                    {
                        SecondBestDistance = Distance;
                    }
                }

                if(BestFeatureID == PANTO_ID_NOT_SET)
                {
                    NumNoBestMatchRejected++;
                    continue;
                }

                NumBestHammingDistanceSamples++;
                SumBestHammingDistance += static_cast<fp64>(BestDistance);
                SquaredSumBestHammingDistance +=
                    static_cast<fp64>(BestDistance) * static_cast<fp64>(BestDistance);

                if(SecondBestDistance != std::numeric_limits<u32>::max())
                {
                    NumSecondBestHammingDistanceSamples++;
                    SumSecondBestHammingDistance += static_cast<fp64>(SecondBestDistance);
                    SquaredSumSecondBestHammingDistance +=
                        static_cast<fp64>(SecondBestDistance) * static_cast<fp64>(SecondBestDistance);
                }

                if(BestDistance >= PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW)
                {
                    NumHammingRejected++;
                    continue;
                }

                if(SecondBestDistance != std::numeric_limits<u32>::max())
                {
                    if(static_cast<fp64>(BestDistance) >= PANTO_MATCHRATIO *
                            static_cast<fp64>(SecondBestDistance))
                    {
                        NumRatioRejected++;
                        continue;
                    }
                }
                typePantoImagePoint& ImagePoint2 = AllImagePoints2[BestFeatureID];
                std::pair<u64, u64> ImagePointIDs(ImagePoint1.ID, ImagePoint2.ID);
                Eigen::Vector4d MapPoint = PROJ_TriangulateDLT(ImagePoint1.Point, ImagePoint2.Point, P1, P2);

                if(!MapPoint.allFinite())
                {
                    NumNonFiniteRejected++;
                    continue;
                }

                const Eigen::Vector3d PointCamera1 = Rt1 * MapPoint;
                const Eigen::Vector3d PointCamera2 = Rt2 * MapPoint;

                if(!PointCamera1.allFinite() || !PointCamera2.allFinite())
                {
                    NumProjectionRejected++;
                    NumProjectionNonFiniteRejected++;
                    continue;
                }

                const bool Depth1Rejected = !PT_IsInfront(MapPoint, Camera1);
                const bool Depth2Rejected = !PT_IsInfront(MapPoint, Camera2);

                if(Depth1Rejected || Depth2Rejected)
                {
                    NumProjectionRejected++;
                    NumRejectedDepthSamples++;
                    SumRejectedDepth1 += PointCamera1.z();
                    SquaredSumRejectedDepth1 += PointCamera1.z() * PointCamera1.z();
                    SumRejectedDepth2 += PointCamera2.z();
                    SquaredSumRejectedDepth2 += PointCamera2.z() * PointCamera2.z();

                    if(Depth1Rejected && Depth2Rejected)
                    {
                        NumBothDepthRejected++;
                    }
                    else if(Depth1Rejected)
                    {
                        NumDepth1OnlyRejected++;
                    }
                    else
                    {
                        NumDepth2OnlyRejected++;
                    }

                    continue;
                }

                const Eigen::Vector3d Projected1 = K * PointCamera1;
                const Eigen::Vector3d Projected2 = K * PointCamera2;

                const Eigen::Vector2d ReprojectedPoint1 =
                {
                    Projected1.x() / Projected1.z(),
                    Projected1.y() / Projected1.z()
                };

                const Eigen::Vector2d ReprojectedPoint2 =
                {
                    Projected2.x() / Projected2.z(),
                    Projected2.y() / Projected2.z()
                };

                const fp64 ReprojectionError1 =
                    (ReprojectedPoint1 - ImagePoint1.Point).squaredNorm();

                const fp64 ReprojectionError2 =
                    (ReprojectedPoint2 - ImagePoint2.Point).squaredNorm();

                const fp64 MaxReprojectionPixelError =
                    sqrt(std::max(ReprojectionError1, ReprojectionError2));

                if(ReprojectionError1 > PANTO_INIT_MAX_REPROJECTION_ERROR ||
                   ReprojectionError2 > PANTO_INIT_MAX_REPROJECTION_ERROR)
                {
                    NumReprojectionRejected++;
                    SumRejectedReprojectionPixelError += MaxReprojectionPixelError;
                    SquaredSumRejectedReprojectionPixelError +=
                        MaxReprojectionPixelError * MaxReprojectionPixelError;

                    if(MaxReprojectionPixelError < PANTO_INIT_MAX_REPROJECTION_ERROR + 2.0)
                    {
                        NumReprojectionRejectedThresholdToThresholdPlusTwo++;
                    }
                    else if(MaxReprojectionPixelError < 10.0)
                    {
                        NumReprojectionRejectedThresholdPlusTwoToTen++;
                    }
                    else if(MaxReprojectionPixelError < 20.0)
                    {
                        NumReprojectionRejectedTenToTwenty++;
                    }
                    else
                    {
                        NumReprojectionRejectedTwentyPlus++;
                    }

                    continue;
                }

                if(PT_IsInfront(MapPoint, Camera1) && PT_IsInfront(MapPoint, Camera2))
                {
                    const typePantoMapPoint NewPoint = PT_CreatePantoMapPoint(MapPoint, ImagePoint1.Descriptor, 
                            KeyFrameIDs, ImagePointIDs, PANTO_ID_NOT_SET, MapAge);
                    const u64 Index = GlobalMapPoints.push_back(NewPoint);
                    GlobalMapPoints[Index].ID = Index;
                    Indexes.push_back(Index);

                    ImagePoint1.MapPointID = Index;
                    ImagePoint2.MapPointID = Index;
                }
                else
                {
                    NumCheiralityRejected++;
                }
            }
            ++FeatureIterator1;
            ++FeatureIterator2;
        }
        else if(FeatureIterator1->first < FeatureIterator2->first)
        {
            FeatureIterator1 = FeatureVector1.lower_bound(FeatureIterator2->first);
        }
        else
        {
            FeatureIterator2 = FeatureVector2.lower_bound(FeatureIterator1->first);
        }
    }

    std::size_t NumImagePoint2Associated = 0;
    std::size_t NumImagePoint2Unassociated = 0;

    for(const typePantoImagePoint& ImagePoint : AllImagePoints2)
    {
        if(ImagePoint.MapPointID == PANTO_ID_NOT_SET)
        {
            NumImagePoint2Unassociated++;
        }
        else
        {
            NumImagePoint2Associated++;
        }
    }

    const std::size_t NumSelectedMatches =
        NumImagePoint1Candidates - NumNoBestMatchRejected;

    const std::size_t NumDescriptorAccepted =
        NumSelectedMatches - NumHammingRejected - NumRatioRejected;

    const std::size_t NumFiniteTriangulations =
        NumDescriptorAccepted - NumNonFiniteRejected;

    const std::size_t NumPositiveDepthTriangulations =
        NumFiniteTriangulations - NumProjectionRejected;

    const std::size_t NumReprojectionAccepted =
        NumPositiveDepthTriangulations - NumReprojectionRejected;

    const fp64 MeanEpipolarDistanceAccepted =
        NumEpipolarAccepted > 0 ?
        SumEpipolarDistanceAccepted / static_cast<fp64>(NumEpipolarAccepted) : 0.0;

    const fp64 EpipolarDistanceVarianceAccepted =
        NumEpipolarAccepted > 0 ?
        SquaredSumEpipolarDistanceAccepted / static_cast<fp64>(NumEpipolarAccepted) -
        MeanEpipolarDistanceAccepted * MeanEpipolarDistanceAccepted : 0.0;

    const std::size_t NumFiniteEpipolarRejected =
        NumEpipolarRejected - NumEpipolarNonFiniteRejected;

    const fp64 MeanEpipolarDistanceRejected =
        NumFiniteEpipolarRejected > 0 ?
        SumEpipolarDistanceRejected / static_cast<fp64>(NumFiniteEpipolarRejected) : 0.0;

    const fp64 EpipolarDistanceVarianceRejected =
        NumFiniteEpipolarRejected > 0 ?
        SquaredSumEpipolarDistanceRejected / static_cast<fp64>(NumFiniteEpipolarRejected) -
        MeanEpipolarDistanceRejected * MeanEpipolarDistanceRejected : 0.0;

    const fp64 EpipolarDistanceStandardDeviationAccepted =
        sqrt(std::max(0.0, EpipolarDistanceVarianceAccepted));

    const fp64 EpipolarDistanceStandardDeviationRejected =
        sqrt(std::max(0.0, EpipolarDistanceVarianceRejected));

    const fp64 MeanBestHammingDistance =
        NumBestHammingDistanceSamples > 0 ?
        SumBestHammingDistance / static_cast<fp64>(NumBestHammingDistanceSamples) : 0.0;

    const fp64 BestHammingDistanceVariance =
        NumBestHammingDistanceSamples > 0 ?
        SquaredSumBestHammingDistance / static_cast<fp64>(NumBestHammingDistanceSamples) -
        MeanBestHammingDistance * MeanBestHammingDistance : 0.0;

    const fp64 BestHammingDistanceStandardDeviation =
        sqrt(std::max(0.0, BestHammingDistanceVariance));

    const fp64 MeanSecondBestHammingDistance =
        NumSecondBestHammingDistanceSamples > 0 ?
        SumSecondBestHammingDistance /
        static_cast<fp64>(NumSecondBestHammingDistanceSamples) : 0.0;

    const fp64 SecondBestHammingDistanceVariance =
        NumSecondBestHammingDistanceSamples > 0 ?
        SquaredSumSecondBestHammingDistance /
        static_cast<fp64>(NumSecondBestHammingDistanceSamples) -
        MeanSecondBestHammingDistance * MeanSecondBestHammingDistance : 0.0;

    const fp64 SecondBestHammingDistanceStandardDeviation =
        sqrt(std::max(0.0, SecondBestHammingDistanceVariance));

    const fp64 MeanRejectedDepth1 =
        NumRejectedDepthSamples > 0 ?
        SumRejectedDepth1 / static_cast<fp64>(NumRejectedDepthSamples) : 0.0;

    const fp64 RejectedDepth1Variance =
        NumRejectedDepthSamples > 0 ?
        SquaredSumRejectedDepth1 / static_cast<fp64>(NumRejectedDepthSamples) -
        MeanRejectedDepth1 * MeanRejectedDepth1 : 0.0;

    const fp64 MeanRejectedDepth2 =
        NumRejectedDepthSamples > 0 ?
        SumRejectedDepth2 / static_cast<fp64>(NumRejectedDepthSamples) : 0.0;

    const fp64 RejectedDepth2Variance =
        NumRejectedDepthSamples > 0 ?
        SquaredSumRejectedDepth2 / static_cast<fp64>(NumRejectedDepthSamples) -
        MeanRejectedDepth2 * MeanRejectedDepth2 : 0.0;

    const fp64 RejectedDepth1StandardDeviation =
        sqrt(std::max(0.0, RejectedDepth1Variance));

    const fp64 RejectedDepth2StandardDeviation =
        sqrt(std::max(0.0, RejectedDepth2Variance));

    const fp64 MeanRejectedReprojectionPixelError =
        NumReprojectionRejected > 0 ?
        SumRejectedReprojectionPixelError /
        static_cast<fp64>(NumReprojectionRejected) : 0.0;

    const fp64 RejectedReprojectionPixelErrorVariance =
        NumReprojectionRejected > 0 ?
        SquaredSumRejectedReprojectionPixelError /
        static_cast<fp64>(NumReprojectionRejected) -
        MeanRejectedReprojectionPixelError * MeanRejectedReprojectionPixelError : 0.0;

    const fp64 RejectedReprojectionPixelErrorStandardDeviation =
        sqrt(std::max(0.0, RejectedReprojectionPixelErrorVariance));

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] KF %llu -> KF %llu: shared nodes = %zu, KF1 unassociated candidates = %zu, KF2 unassociated = %zu, KF2 associated = %zu\n",
            KeyFrame1.ID,
            KeyFrame2.ID,
            NumSharedNodes,
            NumImagePoint1Candidates,
            NumImagePoint2Unassociated,
            NumImagePoint2Associated);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Candidate funnel: KF1 candidates = %zu, selected matches = %zu, descriptor accepted = %zu, finite triangulations = %zu, positive depth = %zu, reprojection accepted = %zu, created = %zu\n",
            NumImagePoint1Candidates,
            NumSelectedMatches,
            NumDescriptorAccepted,
            NumFiniteTriangulations,
            NumPositiveDepthTriangulations,
            NumReprojectionAccepted,
            Indexes.size());

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Per-feature rejects: KF1 invalid = %zu, KF1 associated = %zu, no best match = %zu, hamming = %zu, ratio = %zu, non-finite = %zu, projection/depth = %zu, reprojection = %zu, cheirality = %zu\n",
            NumImagePoint1Invalid,
            NumImagePoint1Associated,
            NumNoBestMatchRejected,
            NumHammingRejected,
            NumRatioRejected,
            NumNonFiniteRejected,
            NumProjectionRejected,
            NumReprojectionRejected,
            NumCheiralityRejected);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Pairwise search work: descriptor comparisons = %zu, KF2 invalid skips = %zu, KF2 associated skips = %zu, not top two = %zu, epipolar rejected = %zu, parallax rejected = %zu\n",
            NumDescriptorComparisons,
            NumImagePoint2Invalid,
            NumImagePoint2AssociatedSkips,
            NumNotTopTwoRejected,
            NumEpipolarRejected,
            NumParallaxRejected);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Epipolar accepted: samples = %zu, mean distance = %lf, standard deviation = %lf\n",
            NumEpipolarAccepted,
            MeanEpipolarDistanceAccepted,
            EpipolarDistanceStandardDeviationAccepted);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Epipolar rejected: finite samples = %zu, non-finite samples = %zu, mean distance = %lf, standard deviation = %lf\n",
            NumFiniteEpipolarRejected,
            NumEpipolarNonFiniteRejected,
            MeanEpipolarDistanceRejected,
            EpipolarDistanceStandardDeviationRejected);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Epipolar rejected bins: [threshold, threshold + 2) = %zu, [threshold + 2, 10) = %zu, [10, 20) = %zu, [20, +inf) = %zu\n",
            NumEpipolarRejectedThresholdToThresholdPlusTwo,
            NumEpipolarRejectedThresholdPlusTwoToTen,
            NumEpipolarRejectedTenToTwenty,
            NumEpipolarRejectedTwentyPlus);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Best Hamming distance: samples = %zu, mean = %lf, standard deviation = %lf\n",
            NumBestHammingDistanceSamples,
            MeanBestHammingDistance,
            BestHammingDistanceStandardDeviation);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Second-best Hamming distance: samples = %zu, mean = %lf, standard deviation = %lf\n",
            NumSecondBestHammingDistanceSamples,
            MeanSecondBestHammingDistance,
            SecondBestHammingDistanceStandardDeviation);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Projection/depth rejected: non-finite = %zu, KF1 depth only = %zu, KF2 depth only = %zu, both depths = %zu\n",
            NumProjectionNonFiniteRejected,
            NumDepth1OnlyRejected,
            NumDepth2OnlyRejected,
            NumBothDepthRejected);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Rejected depths: samples = %zu, KF1 mean = %lf, KF1 standard deviation = %lf, KF2 mean = %lf, KF2 standard deviation = %lf\n",
            NumRejectedDepthSamples,
            MeanRejectedDepth1,
            RejectedDepth1StandardDeviation,
            MeanRejectedDepth2,
            RejectedDepth2StandardDeviation);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Rejected reprojection max pixel error: samples = %zu, mean = %lf, standard deviation = %lf\n",
            NumReprojectionRejected,
            MeanRejectedReprojectionPixelError,
            RejectedReprojectionPixelErrorStandardDeviation);

    LG_Log(LogSeverity::DBG,
            "[KEY_InsertNewMapPoints] Rejected reprojection max pixel error bins: (threshold, threshold + 2) = %zu, [threshold + 2, 10) = %zu, [10, 20) = %zu, [20, +inf) = %zu\n",
            NumReprojectionRejectedThresholdToThresholdPlusTwo,
            NumReprojectionRejectedThresholdPlusTwoToTen,
            NumReprojectionRejectedTenToTwenty,
            NumReprojectionRejectedTwentyPlus);

    return Indexes;
}

void KEY_NonValidKeyFrame(void)
{
    CurrentDescriptors.pop();
}

fp64 KEY_GetLocalMapMedianDepth(const typeKeyFrame& KeyFrame, const std::vector<typePantoMapPoint>& LocalMapPoints)
{
    std::vector<fp64> LocalDepth;

    LocalDepth.reserve((LocalMapPoints.size() + PANTO_LOCAL_MAP_SAMPLE_STRIDE - 1) /
        PANTO_LOCAL_MAP_SAMPLE_STRIDE);

    const typeCameraPose& LocalMapPose = KeyFrame.Camera.Pose;

    for(std::size_t i{}; i < LocalMapPoints.size(); i += PANTO_LOCAL_MAP_SAMPLE_STRIDE)
    {
        const Eigen::Vector3d PointWorld =
            LocalMapPoints[i].Point.head<3>() /
            LocalMapPoints[i].Point.w();

        const Eigen::Vector3d PointCamera =
            LocalMapPose.R * PointWorld + LocalMapPose.t;

        if(PointCamera.z() > 0.0)
        {
            LocalDepth.push_back(PointCamera.z());
        }
    }

    if(LocalDepth.empty())
    {
        LG_Log(LogSeverity::DBG, "[KEY_GetLocalMapMedianDepth] No valid positive-depth local map points\n");
        return 0.0;
    }

    const std::size_t Middle = LocalDepth.size() / 2;

    std::nth_element(
            LocalDepth.begin(),
            LocalDepth.begin() + Middle,
            LocalDepth.end());

    return LocalDepth[Middle];
}

void KEY_IntegrationStep()
{
    typeIMUMeasurement Measurement = IMU_GetMeasurement();
    IMU_IngegrationStep(Measurement);
}

typeNavigationState KEY_PredictPose(typeKeyFrame& PreviousKeyFrame)
{
    typePreIntegrationData PreIntegrationData = IMU_GetLatestPreIntegrationData();
    return IMU_PredictNavigationState(PreviousKeyFrame.NavigationState, PreIntegrationData);
}

void KEYPriv_SolveBootStrapData(void)
{
    assert(BootStrapData.BootStrapDataSolved == false);

    const u64 NumFrames = BootStrapData.NumFrames;
    fp64 MeanVelocityChange = BootStrapData.VelocityChange / NumFrames;
    fp64 MeanTrackingRatio = BootStrapData.LocalMapTrackingRatio / NumFrames;
    fp64 MeanDistanceBetweenFrames = BootStrapData.AcumulatedDistanceTravelled / NumFrames;

    LG_Log( LogSeverity::DBG,
        "[KEYPriv_SolveBootStrapData] NumFrames = %llu, MeanVelocityChange = %f, MeanTrackingRatio = %f, MeanDistanceBetweenFrames = %f\n",
        static_cast<unsigned long long>(NumFrames),
        MeanVelocityChange,
        MeanTrackingRatio,
        MeanDistanceBetweenFrames);

    FuzzyInference.VelocityParamameters.second = MeanVelocityChange * PANTO_KEYFRAME_MEAN_VELOCITY_THRESHOLD_GAIN;
    FuzzyInference.VelocityParamameters.first = MeanVelocityChange;

    FuzzyInference.TrackingRatioParameters.second = MeanTrackingRatio * PANTO_KEYFRAME_MEAN_TRACKING_HIGH_THRESHOLD_GAIN;
    FuzzyInference.TrackingRatioParameters.first = MeanTrackingRatio * PANTO_KEYFRAME_MEAN_TRACKING_LOW_THRESHOLD_GAIN;

    FuzzyInference.AccumulatedDistanceParameters.second = MeanDistanceBetweenFrames * PANTO_KEYFRAME_MEAN_DISTANCE_THRESHOLD_GAIN;
    FuzzyInference.AccumulatedDistanceParameters.first = MeanDistanceBetweenFrames; 

    LG_Log( LogSeverity::DBG,
        "[KEYPriv_SolveBootStrapData] VelocityParameters Low = %f, High = %f\n",
        FuzzyInference.VelocityParamameters.first,
        FuzzyInference.VelocityParamameters.second);

    LG_Log( LogSeverity::DBG,
        "[KEYPriv_SolveBootStrapData] TrackingRatioParameters Low = %f, High = %f\n",
        FuzzyInference.TrackingRatioParameters.first,
        FuzzyInference.TrackingRatioParameters.second);

    LG_Log( LogSeverity::DBG,
        "[KEYPriv_SolveBootStrapData] AccumulatedDistanceParameters Low = %f, High = %f\n",
        FuzzyInference.AccumulatedDistanceParameters.first,
        FuzzyInference.AccumulatedDistanceParameters.second);

    FuzzyInference.MaxRuleThreshold = PANTO_KEYFRAME_FUZZY_MAX_RULE_THRESHOLD;
    FuzzyInference.SpatialTrackingThreshold = PANTO_KEYFRAME_FUZZY_SPATIAL_TRACKING_THRESHOLD;

    LG_Log( LogSeverity::DBG,
        "[KEYPriv_SolveBootStrapData] MaxRuleThreshold = %f, SpatialTrackingThreshold = %f\n",
        FuzzyInference.MaxRuleThreshold,
        FuzzyInference.SpatialTrackingThreshold);

    // FuzzyInference.VelocityParamameters.second = 0.200895 * 1000000;
    // FuzzyInference.VelocityParamameters.first = 0.050224 * 10000;
    //
    // FuzzyInference.TrackingRatioParameters.second = 0.155848;
    // FuzzyInference.TrackingRatioParameters.first = 0.038962;
    //
    // FuzzyInference.AccumulatedDistanceParameters.second = 0.258651;
    // FuzzyInference.AccumulatedDistanceParameters.first = 0.012933; 
    
    FuzzyInference.MaxRuleThreshold = PANTO_KEYFRAME_FUZZY_MAX_RULE_THRESHOLD;
    FuzzyInference.SpatialTrackingThreshold = PANTO_KEYFRAME_FUZZY_SPATIAL_TRACKING_THRESHOLD;

    BootStrapData.BootStrapDataSolved = true;
}

bool KEYPriv_IsKeyFrame(const typeKeyFrameInformation& KeyFrameInformation)
{
    fp64 VelocityMembership = FUZZY_LINEAR_INCREASING_MEMBERSHIP(FuzzyInference.VelocityParamameters.first, 
            FuzzyInference.VelocityParamameters.second, KeyFrameInformation.VelocityChange);

    fp64 DistanceMembership = FUZZY_LINEAR_INCREASING_MEMBERSHIP(FuzzyInference.AccumulatedDistanceParameters.first, 
            FuzzyInference.AccumulatedDistanceParameters.second, KeyFrameInformation.AcumulatedDistanceTravelled);

    fp64 TrackingMembership = FUZZY_LINEAR_DECREASING_MEMBERSHIP(FuzzyInference.TrackingRatioParameters.first, 
            FuzzyInference.TrackingRatioParameters.second, KeyFrameInformation.LocalMapTrackingRatio);

    fp64 MaxMemberShip = FUZZY_UNIONRULEINFERENCE(VelocityMembership, DistanceMembership, TrackingMembership);

    fp64 MinVelTracking = std::min(VelocityMembership, TrackingMembership);
    fp64 MinDistTracking = std::min(DistanceMembership, TrackingMembership);
    fp64 SpatialMembership = FUZZY_UNIONRULEINFERENCE(MinVelTracking, MinDistTracking);
    
    const bool MaxVelocityTriggered = VelocityMembership > FuzzyInference.MaxRuleThreshold;
    const bool MaxDistanceTriggered = DistanceMembership > FuzzyInference.MaxRuleThreshold;
    const bool MaxTrackingTriggered = TrackingMembership > FuzzyInference.MaxRuleThreshold;
    const bool SpatialVelocityTrackingTriggered = MinVelTracking > FuzzyInference.SpatialTrackingThreshold;
    const bool SpatialDistanceTrackingTriggered = MinDistTracking > FuzzyInference.SpatialTrackingThreshold;
    const bool MaxRuleTriggered = MaxVelocityTriggered || MaxDistanceTriggered || MaxTrackingTriggered;
    const bool SpatialRuleTriggered = SpatialVelocityTrackingTriggered || SpatialDistanceTrackingTriggered;
    const bool Decision = MaxRuleTriggered || SpatialRuleTriggered;

    IsKeyFrameStatistics.FuzzyEvaluations++;

    if(Decision)
    {
        IsKeyFrameStatistics.FuzzyTrueDecisions++;
        IsKeyFrameStatistics.MaxRuleTriggered += MaxRuleTriggered;
        IsKeyFrameStatistics.SpatialRuleTriggered += SpatialRuleTriggered;
        IsKeyFrameStatistics.MaxRuleOnly += MaxRuleTriggered && !SpatialRuleTriggered;
        IsKeyFrameStatistics.SpatialRuleOnly += SpatialRuleTriggered && !MaxRuleTriggered;
        IsKeyFrameStatistics.BothRules += MaxRuleTriggered && SpatialRuleTriggered;
        IsKeyFrameStatistics.MaxVelocityTriggered += MaxVelocityTriggered;
        IsKeyFrameStatistics.MaxDistanceTriggered += MaxDistanceTriggered;
        IsKeyFrameStatistics.MaxTrackingTriggered += MaxTrackingTriggered;
        IsKeyFrameStatistics.SpatialVelocityTrackingTriggered += SpatialVelocityTrackingTriggered;
        IsKeyFrameStatistics.SpatialDistanceTrackingTriggered += SpatialDistanceTrackingTriggered;
    }

    LG_Log(LogSeverity::DBG,
            "[KEYPriv_IsKeyFrame] Inputs: VelocityChange = %.6f, AccumulatedDistance = %.6f, TrackingRatio = %.6f\n",
            KeyFrameInformation.VelocityChange,
            KeyFrameInformation.AcumulatedDistanceTravelled,
            KeyFrameInformation.LocalMapTrackingRatio);

    LG_Log(LogSeverity::DBG,
            "[KEYPriv_IsKeyFrame] Memberships: Velocity = %.6f, Distance = %.6f, Tracking = %.6f\n",
            VelocityMembership,
            DistanceMembership,
            TrackingMembership);

    LG_Log(LogSeverity::DBG,
            "[KEYPriv_IsKeyFrame] Rules: MaxMembership = %.6f / %.6f, SpatialMembership = %.6f / %.6f, MinVelTracking = %.6f, MinDistTracking = %.6f\n",
            MaxMemberShip,
            FuzzyInference.MaxRuleThreshold,
            SpatialMembership,
            FuzzyInference.SpatialTrackingThreshold,
            MinVelTracking,
            MinDistTracking);

    LG_Log(LogSeverity::DBG,
            "[KEYPriv_IsKeyFrame] Decision = %s, MaxRuleTriggered = %s, SpatialRuleTriggered = %s\n",
            Decision ? "true" : "false",
            MaxRuleTriggered ? "true" : "false",
            SpatialRuleTriggered ? "true" : "false");

    return Decision;
}

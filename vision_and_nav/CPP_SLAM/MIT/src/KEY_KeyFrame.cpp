#include "KEY_Keyframe.hpp"
#include "Config.hpp"
#include "IMU_IMUReader.hpp"
#include "IMU_PreIntegration.hpp"
#include "KEY_KeyFramePriv.hpp"
#include "PT_PantoMapPoints.hpp"
#include "opencv2/opencv.hpp"

struct typeKeyFrameTimingStatistics
{
    u64 Count = 0;
    fp64 Sum = 0.0;
    fp64 SumSquared = 0.0;
};

static typeKeyFrameTimingStatistics GetKeyFrameTotalTiming{};
static typeKeyFrameTimingStatistics PosePreparationTiming{};
static typeKeyFrameTimingStatistics GetFrameTiming{};
static typeKeyFrameTimingStatistics GetDescriptorsTiming{};
static typeKeyFrameTimingStatistics CreateImagePointsTiming{};
static typeKeyFrameTimingStatistics AssembleKeyFrameTiming{};
static typeKeyFrameTimingStatistics GetKeyFrameOverheadTiming{};
static typeKeyFrameTimingStatistics SetAsKeyFrameNonVocabTransform{};
static typeKeyFrameTimingStatistics SetAsKeyFrameVocabTransform{};

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

typeKeyFrame KEY_CreateKeyFrame(const typeNavigationState& NavState, const typePantoFrame& Frame, const u64 ID)
{
    typeKeyFrame KeyFrame{};

    KeyFrame.Frame = Frame;

    const DBoW3::Vocabulary* Vocab = DBOW3_GetVocabulary();
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

    KeyFrame.ID = ID;
    KeyFrame.ImagePath = Frame.Path;
#if defined(CONFIG_IMU)
    KeyFrame.NavigationState = NavState;
#else
    (void)NavState;
#endif

    const Eigen::Matrix3d& Rbw = NavState.Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw * NavState.Position;

    const Eigen::Matrix3d& CameraR = BodyToCamera.R.transpose() * Rbw;
    const Eigen::Vector3d& Camerat = BodyToCamera.R.transpose() * tbw - BodyToCamera.R.transpose() * BodyToCamera.t;

    KeyFrame.Camera = CM_CreateCam(CameraR, Camerat, Frame.TimeStamp);
    const std::vector<cv::Point2d>& Points = Desc.Points;
    const cv::Mat& Descriptors = Desc.Descriptors;

    typePantoKeypointFrame ImagePoints;
    ImagePoints.ImagePoints.reserve(Points.size());

    for(std::size_t i{}; i < DescriptorVector.size(); i++)
    {
        Eigen::Vector2d Point(Points[i].x, Points[i].y);
        u64 CellX = static_cast<u64>(Point[0]) / PANTO_CELL_SIZE;
        u64 CellY = static_cast<u64>(Point[1]) / PANTO_CELL_SIZE;

        u64 CellIndex = CellY * PANTO_GRID_COLUMNS + CellX;

        typeDescriptor Descriptor;

        std::memcpy(Descriptor.data(), Descriptors.ptr<u8>(i), PANTO_DESCRIPTOR_SIZE);

        typePantoImagePoint CandidateImagePoint = 
        {
            .Point = Point,
            .Descriptor = Descriptor,
            .MapPointID = PANTO_ID_NOT_SET,
            .ID = static_cast<u64>(i),
            .CellID = CellIndex
        };

        ImagePoints.ImagePoints.push_back(std::move(CandidateImagePoint));
        ImagePoints.CellIndexingArray[CellIndex].push_back(i);
    }

    KeyFrame.Points = std::move(ImagePoints);

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
    const DBoW3::Vocabulary* Vocabulary = DBOW3_GetVocabulary();

    DBoW3::BowVector NewBowVector;
    DBoW3::FeatureVector NewFeatureVector;

    Vocabulary->transform(DescriptorVector, NewBowVector, NewFeatureVector, LevelsUp);

    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePointsNoMatch(Descriptors.Points, Descriptors.Descriptors);

    typeCamera PredictedPose = LastKeyFrame.Camera;
    PredictedPose.TimeStamp = Frame.TimeStamp;

    typeKeyFrame KeyFrame = 
    {
        .Points = std::move(ImagePoints),
        .BowVector = std::move(NewBowVector),
        .FeatureVector = std::move(NewFeatureVector),
        .Camera = PredictedPose,
        .ID = 2,
        .ImagePath = std::move(Frame.Path)
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

#if !defined(CONFIG_IMU)
typeKeyFrame KEY_GetKeyFrame(typeCamera& PredictedPose,
        std::vector<typePantoMapPoint>& LastFrameMapPoints)
#else
typeKeyFrame KEY_GetKeyFrame(typeNavigationState& PredictedNavigationState, std::vector<typePantoMapPoint>& LastFrameMapPoints)
#endif
{
    const PantoClock::time_point GetKeyFrameStartTime = PantoClock::now();
    const PantoClock::time_point PosePreparationStartTime = PantoClock::now();

#if defined(CONFIG_IMU)
    const typePose BodyToCamera = CM_GetBodyToSensor(CM_GetIntrinsics());
    const Eigen::Matrix3d Rbw = PredictedNavigationState.Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw * PredictedNavigationState.Position;
    const Eigen::Matrix3d CameraR = BodyToCamera.R.transpose() * Rbw;
    const Eigen::Vector3d Camerat = BodyToCamera.R.transpose() *
        (tbw - BodyToCamera.t);
    typeCamera PredictedPose = CM_CreateCam(CameraR, Camerat, PANTO_TIMESTAMP_NOT_SET);
#endif

    const fp64 PosePreparationTime = std::chrono::duration<fp64>(PantoClock::now() - PosePreparationStartTime).count();

    KEYPriv_AddTimingSample(PosePreparationTiming, PosePreparationTime);

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

    const fp64 GetFrameTime = std::chrono::duration<fp64>(PantoClock::now() - GetFrameStartTime).count();

    KEYPriv_AddTimingSample(GetFrameTiming, GetFrameTime);

    if(Frame.TimeStamp < 0.0f)
    {
        PredictedPose.TimeStamp = -1.0f;

        const PantoClock::time_point AssembleKeyFrameStartTime = PantoClock::now();
        typeKeyFrame KeyFrame =
        {
            .Points = typePantoKeypointFrame{},
            .BowVector = {},
            .FeatureVector = {},
            .Camera = PredictedPose,
#if defined(CONFIG_IMU)
            .NavigationState = PredictedNavigationState,
#endif
            .ID = PANTO_ID_NOT_SET,

            .Frame = Frame,
            .ImagePath = ""
        };
        const fp64 AssembleKeyFrameTime =
            std::chrono::duration<fp64>(PantoClock::now() - AssembleKeyFrameStartTime).count();

        KEYPriv_AddTimingSample(AssembleKeyFrameTiming, AssembleKeyFrameTime);

        const fp64 GetKeyFrameTotalTime = std::chrono::duration<fp64>(PantoClock::now() - GetKeyFrameStartTime).count();
        const fp64 GetKeyFrameOverheadTime = std::max<fp64>(
                0.0,
                GetKeyFrameTotalTime - PosePreparationTime - GetFrameTime - AssembleKeyFrameTime);

        KEYPriv_AddTimingSample(GetKeyFrameTotalTiming, GetKeyFrameTotalTime);
        KEYPriv_AddTimingSample(GetKeyFrameOverheadTiming, GetKeyFrameOverheadTime);

        LG_Log(LogSeverity::DBG,
                "[KEY_GetKeyFrameTiming] total = %.6f s, pose preparation = %.6f s, FR_GetFrame = %.6f s, keyframe assembly = %.6f s, residual overhead = %.6f s, frame invalid\n",
                GetKeyFrameTotalTime,
                PosePreparationTime,
                GetFrameTime,
                AssembleKeyFrameTime,
                GetKeyFrameOverheadTime);

        return KeyFrame;
    }

    PredictedPose.TimeStamp = Frame.TimeStamp;

    const DescRet& Descriptors = Frame.Descriptors;

    const PantoClock::time_point CreateImagePointsStartTime = PantoClock::now();
    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePoints(
            Descriptors.Points, Descriptors.Descriptors,
            LastFrameMapPoints, PredictedPose);
    const fp64 CreateImagePointsTime =
        std::chrono::duration<fp64>(PantoClock::now() - CreateImagePointsStartTime).count();

    KEYPriv_AddTimingSample(CreateImagePointsTiming, CreateImagePointsTime);

    const PantoClock::time_point AssembleKeyFrameStartTime = PantoClock::now();
    typeKeyFrame KeyFrame =
    {
        .Points = std::move(ImagePoints),
        .BowVector = {},
        .FeatureVector = {},
        .Camera = PredictedPose,
#if defined(CONFIG_IMU)
        .NavigationState = PredictedNavigationState,
#endif
        .ID = PANTO_ID_NOT_SET,
        .Frame = Frame,
        .ImagePath = std::move(Frame.Path)
    };
    const fp64 AssembleKeyFrameTime = std::chrono::duration<fp64>(PantoClock::now() - AssembleKeyFrameStartTime).count();

    KEYPriv_AddTimingSample(AssembleKeyFrameTiming, AssembleKeyFrameTime);

    const fp64 GetKeyFrameTotalTime =
        std::chrono::duration<fp64>(PantoClock::now() - GetKeyFrameStartTime).count();
    const fp64 GetKeyFrameOverheadTime = std::max<fp64>(
            0.0,
            GetKeyFrameTotalTime - PosePreparationTime - GetFrameTime -
                CreateImagePointsTime - AssembleKeyFrameTime);

    KEYPriv_AddTimingSample(GetKeyFrameTotalTiming, GetKeyFrameTotalTime);
    KEYPriv_AddTimingSample(GetKeyFrameOverheadTiming, GetKeyFrameOverheadTime);

    LG_Log(LogSeverity::DBG,
            "[KEY_GetKeyFrameTiming] total = %.6f s, pose preparation = %.6f s, FR_GetFrame = %.6f s, PT_CreatePantoImagePoints = %.6f s, keyframe assembly = %.6f s, residual overhead = %.6f s\n",
            GetKeyFrameTotalTime,
            PosePreparationTime,
            GetFrameTime,
            CreateImagePointsTime,
            AssembleKeyFrameTime,
            GetKeyFrameOverheadTime);

    return KeyFrame;
}

void KEY_LogKeyFrameTimingStatistics(void)
{
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/internal total", GetKeyFrameTotalTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/pose preparation", PosePreparationTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/FR_GetFrame", GetFrameTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/EP_GetDescriptors", GetDescriptorsTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/PT_CreatePantoImagePoints", CreateImagePointsTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/keyframe assembly", AssembleKeyFrameTiming);
    KEYPriv_LogTimingStatistics("KEY_GetKeyFrame/residual overhead", GetKeyFrameOverheadTiming);
    KEYPriv_LogTimingStatistics("KEY_SetAsKeyFrame non Vocab", SetAsKeyFrameNonVocabTransform);
    KEYPriv_LogTimingStatistics("KEY_SetAsKeyFrame vocab transform", SetAsKeyFrameVocabTransform);
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
    const auto& StartTimeBeforeVocab = PantoClock::now();
    const u64 ID = KeyFrame.ID;
    LG_Log(LogSeverity::DBG, "[KEY_SetAsKeyFrame] ID = %llu\n", ID);
    std::vector<cv::Mat> DescriptorVector;
    DescriptorVector.reserve(KeyFrame.Points.ImagePoints.active_size());
    for(const typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        // Descriptors belong to the frame. Keeping them in a process-global
        // FIFO lets tracking pop a pending keyframe's descriptors while local
        // mapping is behind, and is also a data race between the two workers.
        DescriptorVector.emplace_back(
                1,
                PANTO_DESCRIPTOR_SIZE,
                CV_8U,
                const_cast<u8*>(ImagePoint.Descriptor.data()));
    }

    const i32 Levels = PANTO_DBOW_LEVELSUP;

    std::vector<typeDescriptor> Descriptors;

    std::vector<i32> ScratchDistances;
    ScratchDistances.reserve(32);

    for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID == PANTO_ID_NOT_SET)
        {
            continue;
        }

        const u64 ImagePointID = ImagePoint.ID;
        typePantoMapPoint& MapPoint = GlobalMapPoints[MapPointID];

#if defined(DEBUG)
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
#endif

        MapPoint.ImagePointIDs.push_back(ImagePointID);
        MapPoint.KeyFrameIDs.push_back(ID);

        Descriptors.clear();
        Descriptors.reserve(MapPoint.KeyFrameIDs.size());

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            if(MapPoint.ImagePointIDs.contains(i))
            {
#if defined(DEBUG)
                assert(MapPoint.KeyFrameIDs.contains(i));
#endif
                const u64 ImagePointID = MapPoint.ImagePointIDs[i];
                const u64 KeyFrameID   = MapPoint.KeyFrameIDs[i];

                Descriptors.push_back(GlobalKeyFrames[KeyFrameID].Points.ImagePoints[ImagePointID].Descriptor);
            }
        }

        if(Descriptors.size() == 1)
        {
            MapPoint.Descriptor = Descriptors[0];
            continue;
        }
        MapPoint.Descriptor = PT_CalculateNewDescriptor(Descriptors, ScratchDistances);
    }
    const auto& TimeBeforeVocab = std::chrono::duration<fp64>(PantoClock::now() - StartTimeBeforeVocab).count();
    KEYPriv_AddTimingSample(SetAsKeyFrameNonVocabTransform, TimeBeforeVocab);
    const auto& TimeStartVocab = PantoClock::now();
    Vocabulary->transform(DescriptorVector, KeyFrame.BowVector, KeyFrame.FeatureVector, Levels);
    const auto& TimeVocab = std::chrono::duration<fp64>(PantoClock::now() - TimeStartVocab).count();
    KEYPriv_AddTimingSample(SetAsKeyFrameVocabTransform, TimeVocab);
}

std::vector<typePantoMapPoint> KEY_InsertNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, const u64 MapAge)
{
    std::vector<typePantoMapPoint> MapPoints;
    MapPoints.reserve(PANTO_NEW_MAPPOINT_RESERVE);

    const Eigen::Matrix3d F21  = EP_GetFundamentalMatrix21(KeyFrame1.Camera.Pose, KeyFrame2.Camera.Pose);
    const Eigen::Matrix3d F12 = F21.transpose();

    typePantoVector<typePantoImagePoint>& AllImagePoints1 = KeyFrame1.Points.ImagePoints;
    typePantoVector<typePantoImagePoint>& AllImagePoints2 = KeyFrame2.Points.ImagePoints;
    std::unordered_set<u64> SelectedImagePointIDs2;

    const DBoW3::FeatureVector& FeatureVector1 = KeyFrame1.FeatureVector;
    const DBoW3::FeatureVector& FeatureVector2 = KeyFrame2.FeatureVector;

    auto FeatureIterator1 = FeatureVector1.begin();
    auto FeatureIterator2 = FeatureVector2.begin();

    std::pair<u64, u64> KeyFrameIDs(KeyFrame1.ID, KeyFrame2.ID);

    const typeCamera& Camera1 = KeyFrame1.Camera;
    const typeCamera& Camera2 = KeyFrame2.Camera;

    const Eigen::Matrix3d& Camera1RTranspose = Camera1.Pose.R.transpose();
    const Eigen::Matrix3d& Camera2RTranspose = Camera2.Pose.R.transpose();

    const Eigen::Matrix3d K = CM_GetIntrinsics()->K;

    Eigen::Matrix<fp64, 3, 4> Rt1 = CM_GetRt(KeyFrame1.Camera);
    Eigen::Matrix<fp64, 3, 4> Rt2 = CM_GetRt(KeyFrame2.Camera);

    const Eigen::Matrix<fp64, 3, 4> P1 = K * Rt1;

    const Eigen::Matrix<fp64, 3, 4> P2 = K * Rt2;

    const fp64 fx = K(0, 0);
    const fp64 fy = K(1, 1);

    const fp64 cx = K(0, 2);
    const fp64 cy = K(1, 2);

    const fp64 fxrep = 1 / fx;
    const fp64 fyrep = 1 / fy;

    
    while(FeatureIterator1 != FeatureVector1.end() && FeatureIterator2 != FeatureVector2.end())
    {
        if(FeatureIterator1->first == FeatureIterator2->first)
        {

            //Feature vector match
            const std::vector<u32>& FeatureIDs1 = FeatureIterator1->second;
            const std::vector<u32>& FeatureIDs2 = FeatureIterator2->second;

            for(const u32& FeatureID1 : FeatureIDs1)
            {
                if(!AllImagePoints1.contains(static_cast<u64>(FeatureID1)))
                {
                    continue;
                }
                typePantoImagePoint& ImagePoint1 = AllImagePoints1[FeatureID1];

                if(ImagePoint1.MapPointID != PANTO_ID_NOT_SET)
                {
                    continue;
                }

                const Eigen::Vector3d Ray1Camera =
                {
                    (ImagePoint1.Point.x() - cx) * fxrep,
                    (ImagePoint1.Point.y() - cy) * fyrep,
                    1.0
                };

                const Eigen::Vector3d Ray1World = Camera1RTranspose * Ray1Camera;

                u32 BestDistance = std::numeric_limits<u32>::max();
                u64 BestFeatureID = PANTO_ID_NOT_SET;

                for(const u32& FeatureID2 : FeatureIDs2)
                {
                    if(!AllImagePoints2.contains(static_cast<u64>(FeatureID2)))
                    {
                        continue;
                    }
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[FeatureID2];

                    if(ImagePoint2.MapPointID != PANTO_ID_NOT_SET ||
                       SelectedImagePointIDs2.contains(ImagePoint2.ID))
                    {
                        continue;
                    }

                    const u32 Distance = PANTO_HammingDistance(ImagePoint1.Descriptor, ImagePoint2.Descriptor);

                    const fp64 MeanEpipolarDistance = EP_CheckEpipolarConstraint(ImagePoint1.Point, ImagePoint2.Point, F21, F12);

                    if(MeanEpipolarDistance >= PANTO_EPIPOLARTRESHOLD)
                    {
                        continue;
                    }

                    const Eigen::Vector3d Ray2Camera =
                    {
                        (ImagePoint2.Point.x() - cx) / fx,
                        (ImagePoint2.Point.y() - cy) / fy,
                        1.0
                    };


                    const Eigen::Vector3d Ray2World = Camera2RTranspose * Ray2Camera;

                    const fp64 CosParallax = Ray1World.normalized().dot(Ray2World.normalized());

                    if(CosParallax <= 0 || CosParallax > PANTO_MAXIMUMCOSPARALLAX)
                    {
                        continue;
                    }

                    if(Distance < BestDistance)
                    {
                        BestDistance = Distance;
                        BestFeatureID = static_cast<u64>(FeatureID2);
                    }
                }

                if(BestFeatureID == PANTO_ID_NOT_SET)
                {
                    continue;
                }

                if(BestDistance >= PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW)
                {
                    continue;
                }

                typePantoImagePoint& ImagePoint2 = AllImagePoints2[BestFeatureID];
                std::pair<u64, u64> ImagePointIDs(ImagePoint1.ID, ImagePoint2.ID);
                Eigen::Vector4d MapPoint = PROJ_TriangulateDLT(ImagePoint1.Point, ImagePoint2.Point, P1, P2);

                if(!MapPoint.allFinite())
                {
                    continue;
                }

                const Eigen::Vector3d PointCamera1 = Rt1 * MapPoint;
                const Eigen::Vector3d PointCamera2 = Rt2 * MapPoint;

                if(!PointCamera1.allFinite() || !PointCamera2.allFinite())
                {
                    continue;
                }

                const bool Depth1Rejected = !PT_IsInfront(MapPoint, Camera1);
                const bool Depth2Rejected = !PT_IsInfront(MapPoint, Camera2);

                if(Depth1Rejected || Depth2Rejected)
                {
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

                const fp64 ReprojectionError1 = (ReprojectedPoint1 - ImagePoint1.Point).squaredNorm();
                const fp64 ReprojectionError2 = (ReprojectedPoint2 - ImagePoint2.Point).squaredNorm();

                if(ReprojectionError1 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED || ReprojectionError2 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED)
                {
                    continue;
                }

                if(PT_IsInfront(MapPoint, Camera1) && PT_IsInfront(MapPoint, Camera2))
                {
                    const typePantoMapPoint NewPoint = PT_CreatePantoMapPoint(MapPoint, ImagePoint1.Descriptor, 
                            KeyFrameIDs, ImagePointIDs, PANTO_ID_NOT_SET, MapAge);
                    MapPoints.push_back(NewPoint);
                    SelectedImagePointIDs2.insert(ImagePoint2.ID);
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
    return MapPoints;
}

void KEY_NonValidKeyFrame(void)
{
    // Descriptors are owned by each frame; there is no cross-thread pending
    // state to discard for a frame that was not selected as a keyframe.
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

#if defined(CONFIG_IMU)
typeIMUMeasurement KEY_IntegrationStep()
{
    typeIMUMeasurement Measurement = IMU_GetMeasurement();
    IMU_IngegrationStep(Measurement);
    return Measurement;
}

typeNavigationState KEY_PredictPose(typeKeyFrame& PreviousKeyFrame)
{
    typePreIntegrationData PreIntegrationData = IMU_GetLatestPreIntegrationData();
    return IMU_PredictNavigationState(PreviousKeyFrame.NavigationState, PreIntegrationData);
}

void KEY_UpdateNavState(typeKeyFrame* KeyFrame)
{
    const Eigen::Matrix4d& TBS = KeyFrame->Camera.Intrinsics->T_BS;

    const Eigen::Matrix3d Rbs = TBS.block<3,3>(0,0);
    const Eigen::Vector3d Tbs = TBS.block<3,1>(0,3);

    const Eigen::Matrix3d& Rsw = KeyFrame->Camera.Pose.R;
    const Eigen::Vector3d& Tsw = KeyFrame->Camera.Pose.t;

    // T_BW = T_BS * T_SW. The navigation state stores the inverse
    // convention: body-to-world rotation and the body origin in world.
    const Eigen::Matrix3d Rbw = Rbs * Rsw;
    const Eigen::Vector3d Tbw = Rbs * Tsw + Tbs;

    typeNavigationState& NavigationState = KeyFrame->NavigationState;
    NavigationState.Rwb = Rbw.transpose();
    NavigationState.Position = -NavigationState.Rwb * Tbw;

    NavigationState.q = Eigen::Quaterniond(NavigationState.Rwb).normalized();
    NavigationState.t = NavigationState.Position;
}

void KEY_ReIntegrate(typeKeyFrame& NextKeyFrame, const std::vector<typeIMUMeasurement>& MeasurementsFromPreviousToRemoved)
{
    typePreIntegration NewIntegrationDataForNext(NextKeyFrame.PreIntegrationData.GyroBias, NextKeyFrame.PreIntegrationData.AccelBias);

    std::vector<typeIMUMeasurement> AllIMUMeasurements;
    AllIMUMeasurements.insert(AllIMUMeasurements.end(), MeasurementsFromPreviousToRemoved.begin(),
                MeasurementsFromPreviousToRemoved.end());

    AllIMUMeasurements.insert(AllIMUMeasurements.end(), NextKeyFrame.Measurements.begin(),
                NextKeyFrame.Measurements.end());

    for(const typeIMUMeasurement& IMUMeasurement : AllIMUMeasurements)
    {
        IMU_IngegrationStep(IMUMeasurement, NewIntegrationDataForNext);
    }

    NextKeyFrame.PreIntegrationData = NewIntegrationDataForNext;
    NextKeyFrame.Measurements = AllIMUMeasurements;
}

#endif // CONFIG_IMU


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
    // FuzzyInference.AccumulatedDistanceParameters.second = 0.10;
    // FuzzyInference.AccumulatedDistanceParameters.first = 0.012933; 

    // FuzzyInference.MaxRuleThreshold = PANTO_KEYFRAME_FUZZY_MAX_RULE_THRESHOLD;
    // FuzzyInference.SpatialTrackingThreshold = PANTO_KEYFRAME_FUZZY_SPATIAL_TRACKING_THRESHOLD;

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

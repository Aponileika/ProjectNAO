#include "KEY_KeyFrame.hpp"
#include "KEY_KeyFramePriv.hpp"

// Since getkeyframe can be reached before setaskeyframe is called this has to be a deque (or just a que really)
static std::queue<cv::Mat> CurrentDescriptors{};

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

    typeCamera PredictedPose = LastKeyFrame.Pose;
    PredictedPose.TimeStamp = Frame.TimeStamp;

    typeKeyFrame KeyFrame = 
    {
        .Points = ImagePoints,
        .BowVector = NewBowVector,
        .FeatureVector = NewFeatureVector,
        .Pose = PredictedPose,
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

                    Eigen::Vector2d ProjectedPoint{};

                    if(!PROJ_Project(MapPoint.Point, ProjectedPoint, KeyFrame.Pose))
                    {
                        continue;
                    }

                    if(VisibleMapPointIDs.insert(MapPoint.ID).second)
                    {
                        MapPoint.NumVisible++;
                    }

                    const fp64 ProjectionError =
                        (ProjectedPoint - ImagePoint1.Point).norm();

                    if(ProjectionError > PANTO_MAPPOINT_MATCH_SEARCH_RADIUS)
                    {
                        continue;
                    }

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

    LG_Log(LogSeverity::DBG, "[KEY_GetKeyFrame] Predicted q = (%f, %f, %f, %f), t = (%f, %f, %f)\n",
        PredictedPose.Parameters.q.w(),
        PredictedPose.Parameters.q.x(),
        PredictedPose.Parameters.q.y(),
        PredictedPose.Parameters.q.z(),
        PredictedPose.Parameters.t[0],
        PredictedPose.Parameters.t[1],
        PredictedPose.Parameters.t[2]);

    typePantoFrame Frame = FR_GetFrame();
    if(Frame.TimeStamp < 0.0f)
    {
        PredictedPose.TimeStamp = -1.0f;
        return
        {
            .Points = typePantoKeypointFrame{},
            .BowVector = {},
            .FeatureVector = {},
            .Pose = PredictedPose,
            .ID = PANTO_ID_NOT_SET,
            .ImagePath = "" 
        };
    }
    PredictedPose.TimeStamp = Frame.TimeStamp;
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);
    CurrentDescriptors.push(Descriptors.Descriptors);
    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePoints(Descriptors.Points, Descriptors.Descriptors, LastFrameMapPoints, PredictedPose,
            GlobalMapPoints);
    typeKeyFrame KeyFrame = 
    {
        .Points = ImagePoints,
        .BowVector = {},
        .FeatureVector = {},
        .Pose = PredictedPose,
        .ID = PANTO_ID_NOT_SET,
        .ImagePath = Frame.Path
    };
    return KeyFrame;
}

bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information)
{
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

        return true;
    }
    if(!BootStrapData.BootStrapDataSolved)
    {
        KEYPriv_SolveBootStrapData();
    }
    if(KEYPriv_IsKeyFrame(Information))
    {
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
    const Eigen::Matrix3d F21  = EP_GetFundamentalMatrix21(KeyFrame1.Pose.Pose, KeyFrame2.Pose.Pose);
    const Eigen::Matrix3d F12 = F21.transpose();

    Eigen::Matrix<fp64, 3, 4> Rt1 = CM_GetRt(KeyFrame1.Pose);
    Eigen::Matrix<fp64, 3, 4> Rt2 = CM_GetRt(KeyFrame2.Pose);

    typePantoVector<typePantoImagePoint>& AllImagePoints1 = KeyFrame1.Points.ImagePoints;
    typePantoVector<typePantoImagePoint>& AllImagePoints2 = KeyFrame2.Points.ImagePoints;

    const DBoW3::FeatureVector& FeatureVector1 = KeyFrame1.FeatureVector;
    const DBoW3::FeatureVector& FeatureVector2 = KeyFrame2.FeatureVector;

    auto FeatureIterator1 = FeatureVector1.begin();
    auto FeatureIterator2 = FeatureVector2.begin();

    std::pair<u64, u64> KeyFrameIDs(KeyFrame1.ID, KeyFrame2.ID);

    const typeCamera& Camera1 = KeyFrame1.Pose;
    const typeCamera& Camera2 = KeyFrame2.Pose;

    const Eigen::Matrix3d K = CM_GetIntrinsics()->K;

    const fp64 fx = K(0, 0);
    const fp64 fy = K(1, 1);

    const fp64 cx = K(0, 2);
    const fp64 cy = K(1, 2);
    
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

                u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                u32 SecondBestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;
                u64 BestFeatureID = PANTO_ID_NOT_SET;

                if(ImagePoint1.MapPointID != PANTO_ID_NOT_SET)
                {
                    continue;
                }

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
                        continue;
                    }
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[FeatureID2];

                    if(ImagePoint2.MapPointID != PANTO_ID_NOT_SET)
                    {
                        continue;
                    }

                    const u32 Distance = PANTO_HammingDistance(ImagePoint1.Descriptor, ImagePoint2.Descriptor);

                    if(Distance >= SecondBestDistance)
                    {
                        continue;
                    }

                    if(!EP_CheckEpipolarConstraint(ImagePoint1.Point, ImagePoint2.Point, F21, F12))
                    {
                        continue;
                    }


                    const Eigen::Vector3d Ray2Camera =
                    {
                        (ImagePoint2.Point.x() - cx) / fx,
                        (ImagePoint2.Point.y() - cy) / fy,
                        1.0
                    };


                    const Eigen::Vector3d Ray2World = Camera2.Pose.R.transpose() * Ray2Camera;

                    const fp64 CosParallax = Ray1World.normalized().dot(Ray2World.normalized());

                    if(CosParallax > PANTO_MAXIMUMCOSPARALLAX)
                    {
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

                if(SecondBestDistance == PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1)
                {
                    continue;
                }
                if(BestFeatureID != PANTO_ID_NOT_SET && BestDistance < PANTO_MATCHRATIO * SecondBestDistance)
                {
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[BestFeatureID];
                    std::pair<u64, u64> ImagePointIDs(ImagePoint1.ID, ImagePoint2.ID);
                    Eigen::Vector4d MapPoint = PROJ_TriangulateDLT(ImagePoint1.Point, ImagePoint2.Point, Rt1, Rt2);

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

    const typeCameraPose& LocalMapPose = KeyFrame.Pose.Pose;

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

    FuzzyInference.VelocityParamameters.second = 0.200895;
    FuzzyInference.VelocityParamameters.first = 0.050224;

    FuzzyInference.TrackingRatioParameters.second = 0.155848;
    FuzzyInference.TrackingRatioParameters.first = 0.038962;

    FuzzyInference.AccumulatedDistanceParameters.second = 0.258651;
    FuzzyInference.AccumulatedDistanceParameters.first = 0.012933; 
    
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
    
    bool Decision = (MaxMemberShip > FuzzyInference.MaxRuleThreshold) || (SpatialMembership > FuzzyInference.SpatialTrackingThreshold);

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
            MaxMemberShip > FuzzyInference.MaxRuleThreshold ? "true" : "false",
            SpatialMembership > FuzzyInference.SpatialTrackingThreshold ? "true" : "false");

    return Decision;
}


#include "../include/INIT_InitializeSLAM.hpp"
#include "INITPriv_InitializeSLAM.hpp"
#include <memory>
#include <random>

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

void INIT_ProcessNewFrame(void)
{
    typePantoFrame Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame.Frame);
    INITPriv_AppendFrame(Descriptors.Points, Descriptors.Descriptors, Frame.TimeStamp);
    INITPriv_MatchHistoricalFrames();
    INITPriv_STRANSAC();
    InitData.EnoughStationaryPointsForInit = INITPriv_EnoughStationaryFeatures();
}

std::pair<typeInitFrame, typeInitFrame> INIT_Initialize(void);
void INIT_DestroyInitData(void);

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

std::vector<typeInitImagePoint> INITPriv_STRANSAC(void)
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

    std::vector<typeInitImagePoint> StationaryFeatures;
    StationaryFeatures.reserve(LatestFrame.ImagePoints.size());
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
            StationaryFeatures.push_back(NewImagePoint);
        }
    }

    return StationaryFeatures;
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

bool INITPriv_EnoughStationaryFeatures(void)
{
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

    DBoW3::BowVector DummyBow;

    Vocabulary.transform(DescriptorVector, DummyBow, InitFrame.FeatureVector, Levels);

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

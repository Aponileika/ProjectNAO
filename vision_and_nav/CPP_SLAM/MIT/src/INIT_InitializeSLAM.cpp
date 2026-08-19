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
                            std::vector<i64> NewTrack(InitData.InitFrames.size(), -1);
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

void INITPriv_STRANSAC(void)
{
    for(typeInitFrame& InitFrame : InitData.InitFrames)
    {
    }
}

std::unique_ptr<ImageToImageMapping> INITPriv_ScoredFAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical)
{
    std::unique_ptr<FundamentalMatrixMapping> Fundamental = std::make_unique<FundamentalMatrixMapping>();
    std::unique_ptr<HomographyMapping> Homography = std::make_unique<HomographyMapping>();

    std::thread HomographyThread(&HomographyMapping::Estimate, Homography.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical));
    std::thread FundamentalThread(&FundamentalMatrixMapping::Estimate, Fundamental.get(), std::cref(PointFrameNew), std::cref(PointFrameHistorical));

    HomographyThread.join();
    FundamentalThread.join();

    if(Fundamental->MaxScore > Homography->MaxScore)
    {
        return Fundamental;
    }

    return Homography;
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

#include "KEY_Keyframe.hpp"
#include "KEY_KeyFramePriv.hpp"

// Since getkeyframe can be reached before setaskeyframe is called this has to be a deque (or just a que really)
static std::queue<cv::Mat> CurrentDescriptors{};

typeKeyFrame KEY_GetKeyFrame(const typeCamera& PredictedPose, const std::vector<typePantoMapPoint>& LastFrameMapPoints)
{
    cv::Mat Frame = FR_GetFrame();
    DescRet Descriptors = EP_GetDescriptors(Frame);
    CurrentDescriptors.push(Descriptors.Descriptors);
    typePantoKeypointFrame ImagePoints = PT_CreatePantoImagePoints(Descriptors.Points, Descriptors.Descriptors, LastFrameMapPoints, PredictedPose);
    typeKeyFrame KeyFrame = {
        .Points = ImagePoints,
        .BowVector = {},
        .FeatureVector = {},
        .Pose = PredictedPose,
        .ID = PANTO_ID_NOT_SET
    };
    return KeyFrame;
}

bool KEY_IsKeyFrame(const typeKeyFrameInformation& Information)
{
    return true;
}

void KEY_SetAsKeyFrame(typeKeyFrame& KeyFrame, const u64& ID, const DBoW3::Vocabulary& Vocabulary)
{
    KeyFrame.ID = ID;
    const cv::Mat& Descriptors = CurrentDescriptors.front();
    std::vector<cv::Mat> DescriptorVector;
    DescriptorVector.reserve(Descriptors.rows);
    for(i32 i{}; i < Descriptors.rows; i++)
    {
        DescriptorVector.push_back(Descriptors.row(i));
    }

    const i32 Levels = PANTO_DBOW_LEVELSUP;
    Vocabulary.transform(DescriptorVector, KeyFrame.BowVector, KeyFrame.FeatureVector, Levels);
    CurrentDescriptors.pop();
}

void KEY_InsertNewMapPoints(typeKeyFrame& KeyFrame1, typeKeyFrame& KeyFrame2, std::vector<typePantoMapPoint>& GlobalMapPoints)
{
    const Eigen::Matrix3d F21  = EP_GetFundamentalMatrix21(KeyFrame1.Pose.Pose, KeyFrame2.Pose.Pose);
    const Eigen::Matrix3d F12 = F21.transpose();

    Eigen::Matrix<fp64, 3, 4> Rt1 = CM_GetRt(KeyFrame1.Pose.Pose);
    Eigen::Matrix<fp64, 3, 4> Rt2 = CM_GetRt(KeyFrame2.Pose.Pose);

    std::vector<typePantoImagePoint>& AllImagePoints1 = KeyFrame1.Points.ImagePoints;
    std::vector<typePantoImagePoint>& AllImagePoints2 = KeyFrame2.Points.ImagePoints;

    const DBoW3::FeatureVector& FeatureVector1 = KeyFrame1.FeatureVector;
    const DBoW3::FeatureVector& FeatureVector2 = KeyFrame2.FeatureVector;

    auto FeatureIterator1 = FeatureVector1.begin();
    auto FeatureIterator2 = FeatureVector2.begin();

    std::pair<u64, u64> KeyFrameIDs(KeyFrame1.ID, KeyFrame2.ID);

    const typeCamera& Camera1 = KeyFrame1.Pose;
    const typeCamera& Camera2 = KeyFrame2.Pose;
    
    while(FeatureIterator1 != FeatureVector1.end() && FeatureIterator2 != FeatureVector2.end())
    {
        if(FeatureIterator1->first == FeatureIterator2->first)
        {
            //Feature vector match
            const std::vector<u32>& FeatureIDs1 = FeatureIterator1->second;
            const std::vector<u32>& FeatureIDs2 = FeatureIterator2->second;

            u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW + 1;

            u64 BestFeatureID = PANTO_ID_NOT_SET;

            for(const u32& FeatureID1 : FeatureIDs1)
            {
                typePantoImagePoint& ImagePoint1 = AllImagePoints1[FeatureID1];
                if(ImagePoint1.MapPointID != PANTO_ID_NOT_SET)
                {
                    continue;
                }

                for(const u32& FeatureID2 : FeatureIDs2)
                {
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[FeatureID2];

                    if(ImagePoint2.MapPointID != PANTO_ID_NOT_SET)
                    {
                        continue;
                    }

                    const u32 Distance = PANTO_HammingDistance(ImagePoint1.Descriptor, ImagePoint2.Descriptor);

                    if(Distance >= BestDistance)
                    {
                        continue;
                    }

                    if(!EP_CheckEpipolarConstraint(ImagePoint1.Point, ImagePoint2.Point, F21, F12))
                    {
                        continue;
                    }

                    BestDistance = Distance;
                    BestFeatureID = static_cast<u64>(FeatureID2);
                }

                if(BestFeatureID != PANTO_ID_NOT_SET)
                {
                    typePantoImagePoint& ImagePoint2 = AllImagePoints2[BestFeatureID];
                    std::pair<u64, u64> ImagePointIDs(ImagePoint1.ID, ImagePoint2.ID);
                    Eigen::Vector4d MapPoint = PROJ_TriangulateDLT(ImagePoint1.Point, ImagePoint2.Point, Rt1, Rt2);
                    if(PT_IsInfront(MapPoint, Camera1) && PT_IsInfront(MapPoint, Camera2))
                    {
                        const u64 MapPointID = GlobalMapPoints.size();
                        const typePantoMapPoint NewPoint = PT_CreatePantoMapPoint(MapPoint, ImagePoint1.Descriptor, 
                                KeyFrameIDs, ImagePointIDs, MapPointID);
                        GlobalMapPoints.push_back(NewPoint);
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
}

void KEY_PopKeyFrame(void)
{
    CurrentDescriptors.pop();
}


#include "CM_Camera.hpp"
#include "Config.hpp"
#include "PANTO_Utils.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "PT_PantoImagePoint.hpp"
#include "MAP_Mapping.hpp"
#include "PT_Types.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <unordered_set>

/**
 * Performs orb stype map point to image point matching, starts by
 * assigning an ID and cell index to all image points, and also converting
 * from opencv format to our preferred format, initializes the map ID as PANTO_ID_NOT_SET 
 * After that it loops through all provided map points, projects them onto the image, if the projection
 * is valid (in front of the camera and valid (u, v) coordinates) it calculates
 * all possible cells based on PANTO_MAPPOINT_MATCH_SEARCH_RADIUS and matches its descriptor
 * with the image descriptors, if the top 2 candidates pass the ratio test or the top
 * candidate is similar enough (distance <  PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
 * it is accepted as a match and its MapPointID is set
 * */
typePantoKeypointFrame PT_CreatePantoImagePoints(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors,
        std::vector<typePantoMapPoint>& CandidateMapPoints,
        const typeCamera& Pose)
{
    const std::size_t NumImagePoints = Points.size();
    assert((static_cast<std::size_t>(Descriptors.rows) == NumImagePoints));

    typePantoKeypointFrame ImagePoints;
    ImagePoints.ImagePoints.reserve(NumImagePoints);

    for(std::size_t i{}; i < NumImagePoints; i++)
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
            .CellID = CellIndex,
#if defined(CONFIG_STEREO)
            .RightCameraMatch = Eigen::Vector2d{}
#endif
        };

        ImagePoints.ImagePoints.push_back(std::move(CandidateImagePoint));
        ImagePoints.CellIndexingArray[CellIndex].push_back(i);
    }

    const u64 NumMatchedMapPoints = MAP_MatchMapPointsToKeyFrame(ImagePoints, CandidateMapPoints, Pose, nullptr);
    LG_Log(LogSeverity::DBG, "[PT_CreatePantoImagePoints] Matched %llu/%zu map points\n",
        static_cast<unsigned long long>(NumMatchedMapPoints),
        CandidateMapPoints.size());

    return ImagePoints;
}

typePantoKeypointFrame PT_CreatePantoImagePointsNoMatch(const std::vector<cv::Point2d>& Points, const cv::Mat& Descriptors)
{
    std::size_t NumImagePoints = Points.size();
    assert((static_cast<std::size_t>(Descriptors.rows) == NumImagePoints));

    typePantoKeypointFrame ImagePoints;
    ImagePoints.ImagePoints.reserve(NumImagePoints);

    for(std::size_t i{}; i < NumImagePoints; i++)
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
            .CellID = CellIndex,
#if defined(CONFIG_STEREO)
            .RightCameraMatch = Eigen::Vector2d{}
#endif
        };

        ImagePoints.ImagePoints.push_back(std::move(CandidateImagePoint));
        ImagePoints.CellIndexingArray[CellIndex].push_back(i);
    }

    return ImagePoints;
}

#if defined(CONFIG_STEREO)
static struct
{
    u64 Frames = 0;
    u64 LeftPoints = 0;
    u64 RightPoints = 0;
    u64 RightDescriptorRows = 0;
    u64 RightPointsIndexed = 0;
    u64 LeftWithRowCandidate = 0;
    u64 LeftWithDisparityCandidate = 0;
    u64 DescriptorAccepted = 0;
    u64 SADRefined = 0;
    u64 Matched = 0;
} StereoMatchData;

// Currently no median rejection of SAD costs.
void PT_StereoMatch(typePantoKeypointFrame&KeyPointFrame, std::vector<cv::Point2d>& RightKeyPoints, const cv::Mat& RightDescriptors,
        const cv::Mat& GrayLeft, const cv::Mat& GrayRight)
{
    StereoMatchData.Frames++;
    StereoMatchData.LeftPoints += KeyPointFrame.ImagePoints.active_size();
    StereoMatchData.RightPoints += RightKeyPoints.size();
    StereoMatchData.RightDescriptorRows += RightDescriptors.rows;

    const typeStereoCameraCalibration StereoCalib = *CM_GetStereoCalibration();
    const fp64 MaxDisparity = StereoCalib.K.row(0)[0] * StereoCalib.Baseline / DENSE_MAP_MIN_DEPTH;
    const fp64 MinDisparity = StereoCalib.K.row(0)[0] * StereoCalib.Baseline / DENSE_MAP_MAX_DEPTH;
    std::vector<std::vector<std::pair<cv::Point2d, cv::Mat>>> FeaturesByRow;
    FeaturesByRow.resize(PANTO_IMAGE_HEIGHT);

    std::vector<std::vector<bool>> Inserted;
    Inserted.resize(PANTO_IMAGE_HEIGHT);

    for(std::size_t i{}; i < RightKeyPoints.size(); i++)
    {
        const cv::Point2d& RightPoint = RightKeyPoints[i];
        if(!std::isfinite(RightPoint.y) || RightPoint.y < 0 ||
                RightPoint.y >= static_cast<fp64>(PANTO_IMAGE_HEIGHT))
        {
            continue;
        }
        const u64 y = static_cast<u64>(std::floor(RightPoint.y));
        FeaturesByRow[y].push_back(std::pair(RightPoint, RightDescriptors.row(i)));
        Inserted[y].push_back(false);
        StereoMatchData.RightPointsIndexed++;
    }

    for(typePantoImagePoint& LeftImagePoint : KeyPointFrame.ImagePoints)
    {
        const i64 y = static_cast<i64>(std::floor(LeftImagePoint.Point.y()));
        i64 RowSearchStart = y - PANTO_ROW_SEARCH_STEREO;
        if(RowSearchStart < 0)
        {
            RowSearchStart = 0;
        }
        i64 RowSearchEnd = y + PANTO_ROW_SEARCH_STEREO;
        if(RowSearchEnd >= static_cast<i64>(PANTO_IMAGE_HEIGHT))
        {
            RowSearchEnd = PANTO_IMAGE_HEIGHT;
        }

        const fp64 LeftX = LeftImagePoint.Point.x();

        u32 BestHamming = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;
        u32 SecondBestHamming = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;
        u64 BestRowIdx = RowSearchStart;
        // Note this is not actually a column
        u64 BestColIdx = 0;

        cv::Point2d BestPoint = cv::Point2d{};
        bool HasRowCandidate = false;
        bool HasDisparityCandidate = false;

        for(i64 i = RowSearchStart; i < RowSearchEnd; i++)
        {
            for(std::size_t j{}; j < FeaturesByRow[i].size(); j++)
            {
                if(Inserted[i][j])
                {
                    continue;
                }
                HasRowCandidate = true;
                const std::pair<cv::Point2d, cv::Mat>& ImageFeature = FeaturesByRow[i][j];
                const fp64 Disparity = LeftX - ImageFeature.first.x;
                if(Disparity <= MinDisparity || Disparity >= MaxDisparity)
                {
                    continue;
                }
                HasDisparityCandidate = true;
                typeDescriptor Descriptor;
                std::memcpy(Descriptor.data(), ImageFeature.second.ptr<u8>(0), PANTO_DESCRIPTOR_SIZE);
                const u32 HammingDistance = PANTO_HammingDistance(LeftImagePoint.Descriptor, Descriptor);
                if(HammingDistance < BestHamming)
                {
                    SecondBestHamming = HammingDistance;
                    BestHamming = HammingDistance;
                    BestRowIdx = i;
                    BestColIdx = j;
                    BestPoint = ImageFeature.first;
                }
                else if(HammingDistance < SecondBestHamming)
                {
                    SecondBestHamming = HammingDistance;
                }
            }
        }
        StereoMatchData.LeftWithRowCandidate += HasRowCandidate;
        StereoMatchData.LeftWithDisparityCandidate += HasDisparityCandidate;
        // Two matches and passes ratio test || besthamming < low_threshold
        if((static_cast<fp64>(BestHamming) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestHamming)
                && SecondBestHamming != PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1)
                || BestHamming < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD_LOW )
        {
            StereoMatchData.DescriptorAccepted++;
            const i64 RowCenterLeft = static_cast<i64>(std::floor(LeftImagePoint.Point.y()));
            const i64 ColCenterLeft = static_cast<i64>(std::floor(LeftImagePoint.Point.x()));

            const i64 RowCenterRight = static_cast<i64>(std::floor(BestPoint.y));
            const i64 ColCenterRight = static_cast<i64>(std::floor(BestPoint.x));

            fp64 SADRefinedDelta = PANTO_ZeroMeanSAD(GrayLeft, GrayRight, RowCenterLeft, ColCenterLeft, RowCenterRight, ColCenterRight);
            if(!std::isfinite(SADRefinedDelta))
            {
                continue;
            }
            StereoMatchData.SADRefined++;

            BestPoint.x += SADRefinedDelta;
            const fp64 RefinedDisparity = LeftX - BestPoint.x;
            if(RefinedDisparity <= MinDisparity || RefinedDisparity >= MaxDisparity)
            {
                continue;
            }
            LeftImagePoint.RightCameraMatch.x() = BestPoint.x;
            LeftImagePoint.RightCameraMatch.y() = BestPoint.y;
            LeftImagePoint.IsMatched = true;
            Inserted[BestRowIdx][BestColIdx] = true;
            StereoMatchData.Matched++;
        }
    }
}

void PT_LogStereoMatchData(void)
{
    LG_Log(LogSeverity::DATA,
            "\n"
            "===================== STEREO MATCH DATA =====================\n"
            " Frames processed                   : %llu\n"
            " Left image points                  : %llu\n"
            " Right keypoints supplied           : %llu\n"
            " Right descriptor rows              : %llu\n"
            " Right keypoints indexed            : %llu\n"
            " Left points with row candidate     : %llu\n"
            " Left points with disparity candidate: %llu\n"
            " Descriptor matches accepted        : %llu\n"
            " Finite SAD refinements             : %llu\n"
            " Final left-right matches           : %llu\n"
            "=============================================================\n",
            static_cast<unsigned long long>(StereoMatchData.Frames),
            static_cast<unsigned long long>(StereoMatchData.LeftPoints),
            static_cast<unsigned long long>(StereoMatchData.RightPoints),
            static_cast<unsigned long long>(StereoMatchData.RightDescriptorRows),
            static_cast<unsigned long long>(StereoMatchData.RightPointsIndexed),
            static_cast<unsigned long long>(StereoMatchData.LeftWithRowCandidate),
            static_cast<unsigned long long>(StereoMatchData.LeftWithDisparityCandidate),
            static_cast<unsigned long long>(StereoMatchData.DescriptorAccepted),
            static_cast<unsigned long long>(StereoMatchData.SADRefined),
            static_cast<unsigned long long>(StereoMatchData.Matched));
}
#endif // CONFIG_STEREO

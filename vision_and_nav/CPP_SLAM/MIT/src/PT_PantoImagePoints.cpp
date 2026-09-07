#include "PT_PantoImagePoint.hpp"
#include "MAP_Mapping.hpp"
#include <array>
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
            .CellID = CellIndex
        };

        ImagePoints.ImagePoints.push_back(CandidateImagePoint);
        ImagePoints.CellIndexingArray[CellIndex].push_back(i);
    }

    const u64 NumMatchedMapPoints = MAP_MatchMapPointsToKeyFrame(
            ImagePoints, CandidateMapPoints, Pose, nullptr);
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
            .CellID = CellIndex
        };

        ImagePoints.ImagePoints.push_back(CandidateImagePoint);
        ImagePoints.CellIndexingArray[CellIndex].push_back(i);
    }

    return ImagePoints;
}

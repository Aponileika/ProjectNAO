#include "PT_PantoImagePoint.hpp"
#include <array>
#include <cstddef>
#include <cstring>

/**
 * Performs orb stype map point to image point matching, starts by
 * assigning an ID and cell index to all image points, and also converting
 * from opencv format to our preferred format, initializes the map ID as PANTO_ID_NOT_SET 
 * After that it loops through all provided map points, projects them onto the image, if the projection
 * is valid (in front of the camera and valid (u, v) coordinates) it calculates
 * all possible cells based on PANTO_MAPPOINT_MATCH_SEARCH_AREA and matches its descriptor
 * with the image descriptors, if the top 2 candidates pass the ratio test or the top
 * candidate is similar enough (distance <  PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
 * it is accepted as a match and its MapPointID is set
 * */
typePantoKeypointFrame PT_CreatePantoImagePoints(std::vector<cv::Point2d> Points, 
        cv::Mat Descriptors, std::vector<typePantoMapPoint> CandidateMapPoints, typeCamera Pose)
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

    (void) PT_MatchMapPointsToKeyFrame(ImagePoints, CandidateMapPoints, Pose);

    return ImagePoints;
}

u64 PT_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame, const std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose)
{
    std::size_t NumMapPoints = MapPoints.size();
    u64 NumTrackedMapPoints = 0;

    for(std::size_t i{}; i < NumMapPoints; i++)
    {
        Eigen::Vector4d MapPoint = MapPoints[i].Point;
        Eigen::Vector2d CandidateImagePoint = {};
        if(PROJ_Project(MapPoint, CandidateImagePoint, Pose))
        {
            const fp64 u = CandidateImagePoint[0];
            const fp64 v = CandidateImagePoint[1];

            const u64 MinCellX = static_cast<u64>((u - PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);
            const u64 MaxCellX = static_cast<u64>((u + PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);

            const u64 MinCellY = static_cast<u64>((v - PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);
            const u64 MaxCellY = static_cast<u64>((v + PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);

            const typeDescriptor& MapPointDescriptor = MapPoints[i].Descriptor;

            std::vector<std::pair<typePantoImagePoint, u32>> Top2Candidates(2);

            Top2Candidates[0] = std::make_pair(typePantoImagePoint{}, PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1);
            Top2Candidates[0] = std::make_pair(typePantoImagePoint{}, PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1);

            for(u64 j(MinCellY); j < MaxCellY; j++)
            {
                for(u64 k(MinCellX); k < MaxCellX; k++)
                {
                    std::vector<u64>& LocalImagePoints = KeyFrame.CellIndexingArray[j * PANTO_GRID_COLUMNS + k];
                    for(const u64& ImagePointIdx : LocalImagePoints)
                    {
                        typePantoImagePoint ImagePoint = KeyFrame.ImagePoints[ImagePointIdx];
                        const typeDescriptor& ImagePointDescriptor = ImagePoint.Descriptor;
                        const u32 HammingDistance = PANTO_HammingDistance(MapPointDescriptor, ImagePointDescriptor);
                        if(HammingDistance < Top2Candidates[0].second) 
                        {
                            Top2Candidates[0].first = ImagePoint;
                            Top2Candidates[0].second = HammingDistance;
                        }
                        else if(HammingDistance < Top2Candidates[1].second)
                        {
                            Top2Candidates[1].first = ImagePoint;
                            Top2Candidates[1].second = HammingDistance;
                        }
                    }
                }
            }
            if(Top2Candidates[1].second == PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1) continue;
            if((static_cast<fp64>(Top2Candidates[0].second) < PANTO_MATCHRATIO * static_cast<fp64>(Top2Candidates[1].second))
                    || (Top2Candidates[0].second < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD))
            {
                const typePantoImagePoint& TopCandidate = Top2Candidates[0].first;
                KeyFrame[TopCandidate.CellID][TopCandidate.ID].MapPointID = MapPoints[i].ID;
                NumTrackedMapPoints++;
            }
        }
    }
    return NumTrackedMapPoints;
}


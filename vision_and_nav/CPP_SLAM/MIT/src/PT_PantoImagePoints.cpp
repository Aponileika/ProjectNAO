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
std::array<std::vector<typePantoImagePoint>, PANTO_CELL_SIZE*PANTO_CELL_SIZE> PT_CreatePantoImagePoints(std::vector<cv::Point2d> Points, 
        cv::Mat Descriptors, std::vector<typePantoMapPoint> CandidateMapPoints, Camera Pose)
{
    std::size_t NumImagePoints = Points.size();
    assert((static_cast<std::size_t>(Descriptors.rows) == NumImagePoints));

    std::array<std::vector<typePantoImagePoint>, PANTO_CELL_SIZE*PANTO_CELL_SIZE> ImagePoints;

    for(std::size_t i{}; i < NumImagePoints; i++)
    {
        Eigen::Vector2d Point(Points[i].x, Points[i].y);
        u64 CellX = static_cast<u64>(Point[0]) / PANTO_CELL_SIZE;
        u64 CellY = static_cast<u64>(Point[1]) / PANTO_CELL_SIZE;

        u64 CellIndex = CellY * PANTO_GRID_COLUMNS + CellX;

        typeDescriptor Descriptor;

        std::memcpy(Descriptor.data(), Descriptors.ptr<u8>(i), PANTO_DESCRIPTOR_SIZE);

        u64 ID =  ImagePoints[CellIndex].size();

        typePantoImagePoint CandidateImagePoint = 
        {
            .Point = Point,
            .Descriptor = Descriptor,
            .MapPointID = PANTO_ID_NOT_SET,
            .ID = static_cast<u64>(i),
            .CellID = CellIndex
        };

        ImagePoints[CellIndex].push_back(CandidateImagePoint);
    }

    std::size_t NumMapPoints = CandidateMapPoints.size();

    for(std::size_t i{}; i < NumMapPoints; i++)
    {
        Eigen::Vector4d CandidateMapPoint = CandidateMapPoints[i].Point;
        Eigen::Vector2d CandidateImagePoint = {};
        if(PROJ_Project(CandidateMapPoint, CandidateImagePoint, Pose))
        {
            const fp64 u = CandidateImagePoint[0];
            const fp64 v = CandidateImagePoint[1];

            const u64 MinCellX = static_cast<u64>((u - PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);
            const u64 MaxCellX = static_cast<u64>((u + PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);

            const u64 MinCellY = static_cast<u64>((v - PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);
            const u64 MaxCellY = static_cast<u64>((v + PANTO_MAPPOINT_MATCH_SEARCH_AREA) / PANTO_CELL_SIZE);

            const typeDescriptor& MapPointDescriptor = CandidateMapPoints[i].Descriptor;

            std::vector<std::pair<typePantoImagePoint, u32>> Top2Candidates = {};

            for(u64 j(MinCellY); j < MaxCellY; j++)
            {
                for(u64 k(MinCellX); k < MaxCellX; k++)
                {
                    std::vector<typePantoImagePoint>& LocalImagePoints = ImagePoints[j * PANTO_GRID_COLUMNS + k];
                    for(const typePantoImagePoint& ImagePoint : LocalImagePoints)
                    {
                        const typeDescriptor& ImagePointDescriptor = ImagePoint.Descriptor;
                        const u32 HammingDistance = PANTO_HammingDistance(MapPointDescriptor, ImagePointDescriptor);
                        if(Top2Candidates.size() < 2)
                        {
                            Top2Candidates.push_back(std::pair(ImagePoint, HammingDistance));
                        }
                        else if(HammingDistance < Top2Candidates[0].second) 
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
            if(static_cast<fp64>(Top2Candidates[0].second) < PANTO_MATCHRATIO * static_cast<fp64>(Top2Candidates[1].second))
            {
                const typePantoImagePoint& TopCandidate = Top2Candidates[0].first;
                ImagePoints[TopCandidate.CellID][TopCandidate.ID].MapPointID = CandidateMapPoints[i].ID;
            }
            else if (Top2Candidates[0].second < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
            {
                const typePantoImagePoint& TopCandidate = Top2Candidates[0].first;
                ImagePoints[TopCandidate.CellID][TopCandidate.ID].MapPointID = CandidateMapPoints[i].ID;
            }
        }
    }
}

#include "PT_PantoImagePoint.hpp"
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
        const cv::Mat& Descriptors, std::vector<typePantoMapPoint>& CandidateMapPoints, const typeCamera& Pose,
        typePantoVector<typePantoMapPoint>& GlobalMapPoints)
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

    const u64 NumMatchedMapPoints = PT_MatchMapPointsToKeyFrame(ImagePoints, CandidateMapPoints, Pose, GlobalMapPoints);
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

u64 PT_MatchMapPointsToKeyFrame(typePantoKeypointFrame& KeyFrame, std::vector<typePantoMapPoint>& MapPoints, const typeCamera& Pose,
        typePantoVector<typePantoMapPoint>& GlobalMapPoints)
{
    std::unordered_set<u64> UniqueMapPointIDs;

#if defined(DEBUG)
    for(const typePantoMapPoint& MapPoint : MapPoints)
    {
        assert(UniqueMapPointIDs.insert(MapPoint.ID).second);
    }
#endif

    std::unordered_set<u64> AssociatedMapPointIDs;

    for(const typePantoImagePoint& ImagePoint : KeyFrame.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
        {
            AssociatedMapPointIDs.insert( ImagePoint.MapPointID);
        }
    }

    std::size_t NumMapPoints = MapPoints.size();
    u64 NumTrackedMapPoints = 0;

    u64 NumProjectedMapPoints = 0;
    u64 NumCandidateImagePoints = 0;
    u64 NumWithTwoCandidates = 0;

    for(std::size_t i{}; i < NumMapPoints; i++)
    {
        Eigen::Vector4d MapPoint = MapPoints[i].Point;
        Eigen::Vector2d CandidateImagePoint = {};
        const u64 MapPointID = MapPoints[i].ID;

        if(AssociatedMapPointIDs.contains(MapPointID))
        {
            continue;
        }

        if(PROJ_Project(MapPoint, CandidateImagePoint, Pose))
        {
            NumProjectedMapPoints++;
            MapPoints[i].NumVisible++;
            GlobalMapPoints[MapPoints[i].ID].NumVisible++;

            const fp64 u = CandidateImagePoint[0];
            const fp64 v = CandidateImagePoint[1];

            const fp64 Radius = PANTO_MAPPOINT_MATCH_SEARCH_RADIUS;

            const fp64 MinU = u - Radius;
            const fp64 MaxU = u + Radius;
            const fp64 MinV = v - Radius;
            const fp64 MaxV = v + Radius;

            const i64 MinCellX = std::max<i64>( 0, static_cast<i64>((u - PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MaxCellX = std::min<i64>( PANTO_GRID_COLUMNS - 1, static_cast<i64>((u + PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MinCellY = std::max<i64>( 0, static_cast<i64>((v - PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));
            const i64 MaxCellY = std::min<i64>( PANTO_GRID_ROWS - 1, static_cast<i64>((v + PANTO_MAPPOINT_MATCH_SEARCH_RADIUS) / PANTO_CELL_SIZE));

            const typeDescriptor& MapPointDescriptor = MapPoints[i].Descriptor;
            u32 BestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;

            u32 SecondBestDistance = PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD + 1;

            u64 BestImagePointID = PANTO_ID_NOT_SET;

            for(i64 j(MinCellY); j <= MaxCellY; j++)
            {
                for(i64 k(MinCellX); k <= MaxCellX; k++)
                {
                    std::vector<u64>& LocalImagePoints = KeyFrame.CellIndexingArray[j * PANTO_GRID_COLUMNS + k];

                    for(const u64& ImagePointIdx : LocalImagePoints)
                    {
                        NumCandidateImagePoints++;

                        typePantoImagePoint& ImagePoint = KeyFrame.ImagePoints[ImagePointIdx];

                        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
                        {
                            continue;
                        }

                        const fp64 ImageU = ImagePoint.Point[0];
                        const fp64 ImageV = ImagePoint.Point[1];

                        if(
                                ImageU < MinU ||
                                ImageU > MaxU ||
                                ImageV < MinV ||
                                ImageV > MaxV)
                        {
                            continue;
                        }

                        const typeDescriptor& ImagePointDescriptor = ImagePoint.Descriptor;

                        const u32 HammingDistance = PANTO_HammingDistance( MapPointDescriptor, ImagePointDescriptor);

                        if(HammingDistance < BestDistance)
                        {
                            SecondBestDistance = BestDistance;

                            BestDistance = HammingDistance;
                            BestImagePointID = ImagePoint.ID;
                        }
                        else if(HammingDistance < SecondBestDistance)
                        {
                            SecondBestDistance = HammingDistance;
                        }
                    }
                }
            }
            if(SecondBestDistance > PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
            {
                continue;
            }

            NumWithTwoCandidates++;

            if( static_cast<fp64>(BestDistance) < PANTO_MATCHRATIO * static_cast<fp64>(SecondBestDistance)
                    && BestDistance < PANTO_HAMMING_DISTANCE_MATCH_THRESHOLD)
            {
                KeyFrame.ImagePoints[ BestImagePointID].MapPointID = MapPointID;

                AssociatedMapPointIDs.insert( MapPointID);

                NumTrackedMapPoints++;

                MapPoints[i].NumFound++;
                GlobalMapPoints[MapPointID].NumFound++;
            }
        }
    }

    LG_Log(LogSeverity::DBG, "[PT_MatchMapPointsToKeyFrame] Projected %llu/%zu map points, checked %llu image points, %llu had two candidates, %llu matched\n",
        static_cast<unsigned long long>(NumProjectedMapPoints),
        NumMapPoints,
        static_cast<unsigned long long>(NumCandidateImagePoints),
        static_cast<unsigned long long>(NumWithTwoCandidates),
        static_cast<unsigned long long>(NumTrackedMapPoints));

    return NumTrackedMapPoints;
}


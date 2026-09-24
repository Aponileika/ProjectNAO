#include "DENSE_DenseMapping.hpp"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "LG_Logging.hpp"
#include "MAP_Mapping.hpp"
#include "PANTOVEC_PantoVector.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <unordered_map>

static cv::Ptr<cv::StereoSGBM> DENSEPriv_InitSGBM(void);
typeDenseKeyFrameMap DENSEPriv_CalculateDenseKeyFrameMap(const typeDenseData& DenseData);
static void DENSEPriv_WriteDenseWCS(const typeDenseMapData& DenseData, const typeDenseLocaMapData& LocalMapData, typeDenseVoxelOccupancyMap& VoxelMap);
static cv::Mat DENSEPriv_GetDisparityVisualization(const cv::Mat& Disparity16, const cv::StereoSGBM& StereoSGBM);
static bool DENSEPriv_WritePLY(const std::vector<Eigen::Vector4f>& Points, const std::string& Path);
std::vector<Eigen::Vector4f> DENSEPriv_GetDenseMap(const typeDenseMapData& DenseData, const typeGlobalMap& GlobalMap);
static void DENSEPriv_LogTimingData(void);

static u64 NumPairsCalculated = 0;
static fp64 SumQueueWait = 0.0;
static fp64 SumDenseIteration = 0.0;
static fp64 SumTotalDenseMapCalc = 0.0;
static fp64 SumRemapAndDisparity = 0.0;
static fp64 SumDisparityTo3D = 0.0;
static fp64 SumDisparityVizEnqueue = 0.0;
static fp64 SumDenseMapStore = 0.0;
static fp64 SumWriteDenseWCS = 0.0;
static fp64 SumRollingGridClear = 0.0;
static fp64 SumWorldProjectionAndOccupancy = 0.0;
static fp64 SumWCSPointPublish = 0.0;
static fp64 SumOccupancyPublish = 0.0;

namespace
{
    std::vector<Eigen::Vector3f> WCSMapPoints;
    std::mutex WCSMapMutex;
    typeDenseVoxelOccupancyMap RollingVoxelOccupancyMap;
    std::mutex RollingVoxelOccupancyMapMutex;
}

void DENSE_DenseMapping(typeDenseMapData& MapData)
{
    typeDenseLocaMapData DenseData;
    typeDenseVoxelOccupancyMap OccupancyMap{};
    OccupancyMap.RollingOccupancy.resize(DENSE_NUM_ROLLING_VOXELS);
    while(true)
    {
        const PantoClock::time_point QueueWaitStart = PantoClock::now();
        bool DequeRet = MapData.DenseQueue->deque(DenseData);
        const fp64 QueueWait = std::chrono::duration<fp64>(
                PantoClock::now() - QueueWaitStart).count();
        // False means stop is signalled.
        if(!DequeRet)
        {
            break;
        }
        SumQueueWait += QueueWait;

        const PantoClock::time_point IterationStart = PantoClock::now();
        const PantoClock::time_point StartTimeDense = PantoClock::now();
        typeDenseKeyFrameMap DenseMap = DENSEPriv_CalculateDenseKeyFrameMap(DenseData.DenseData);
        SumTotalDenseMapCalc += std::chrono::duration<fp64>(PantoClock::now() - StartTimeDense).count();

        const PantoClock::time_point VizEnqueueStart = PantoClock::now();
        MapData.VizQueue->enque(DenseMap.DisparityColored);
        SumDisparityVizEnqueue += std::chrono::duration<fp64>(
                PantoClock::now() - VizEnqueueStart).count();

        const PantoClock::time_point DenseMapStoreStart = PantoClock::now();
        const u64 KeyFrameID = DenseMap.KeyFrameID;
        MapData.KeyFrameMaps.insert_or_assign(KeyFrameID, std::move(DenseMap));
        SumDenseMapStore += std::chrono::duration<fp64>(
                PantoClock::now() - DenseMapStoreStart).count();

        const PantoClock::time_point WriteDenseWCSStart = PantoClock::now();
        DENSEPriv_WriteDenseWCS(MapData, DenseData, OccupancyMap);
        SumWriteDenseWCS += std::chrono::duration<fp64>(
                PantoClock::now() - WriteDenseWCSStart).count();

        SumDenseIteration += std::chrono::duration<fp64>(
                PantoClock::now() - IterationStart).count();
        NumPairsCalculated++;
    }

    LG_EnableDataSummaryLoggingForCurrentThread(true);
    DENSEPriv_LogTimingData();

    MapData.VizQueue->stop();
}

static cv::Ptr<cv::StereoSGBM> DENSEPriv_InitSGBM(void)
{
    cv::Ptr<cv::StereoSGBM> StereoSGBM = cv::StereoSGBM::create(
            OPENCV_SGBM_MIN_DISPARITY, OPENCV_SGBM_NUM_DISPARITIES, OPENCV_SGBM_BLOCK_SIZE);

    StereoSGBM->setP1(OPENCV_SGBM_P1);
    StereoSGBM->setP2(OPENCV_SGBM_P2);

    StereoSGBM->setPreFilterCap(OPENCV_SGBM_PRE_FILTER_CAP);
    StereoSGBM->setUniquenessRatio(OPENCV_SGBM_UNIQUENESS_RATIO);
    StereoSGBM->setDisp12MaxDiff(OPENCV_SGBM_DISP12_MAX_DIFF);
    StereoSGBM->setSpeckleWindowSize(OPENCV_SGBM_SPECKLE_WINDOW_SIZE);
    StereoSGBM->setSpeckleRange(OPENCV_SGBM_SPECKLE_RANGE);
    StereoSGBM->setMode(OPENCV_SGBM_MODE);

    return StereoSGBM;
}

typeDenseKeyFrameMap DENSEPriv_CalculateDenseKeyFrameMap(const typeDenseData& DenseData)
{
    const static typeStereoCameraCalibration StereoCalib = *CM_GetStereoCalibration();
    static cv::Ptr<cv::StereoSGBM> StereoSGBM = DENSEPriv_InitSGBM();
    const static fp64 fx = StereoCalib.K.row(0)[0];
    const static fp64 fy = StereoCalib.K.row(1)[1];
    const static fp64 cx = StereoCalib.K.row(0)[2];
    const static fp64 cy = StereoCalib.K.row(1)[2];

    const static fp64 fxrep = 1 / fx;
    const static fp64 fyrep = 1 / fy;
    const static fp64 fxtimesB = StereoCalib.Baseline * fx;

    const cv::Mat& Left = DenseData.LeftImage;
    const cv::Mat& Right = DenseData.RightImage;
    cv::Mat RectifiedLeft;
    cv::Mat RectifiedRight;

    const auto& StartTimeDisparity = PantoClock::now();
    cv::remap(Left, RectifiedLeft, StereoCalib.Map0X, StereoCalib.Map0Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(Right, RectifiedRight, StereoCalib.Map1X, StereoCalib.Map1Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat Disparity16;

    StereoSGBM->compute(RectifiedLeft, RectifiedRight, Disparity16);

    SumRemapAndDisparity += std::chrono::duration<fp64>(PantoClock::now() - StartTimeDisparity).count();

    cv::Mat DisparityColored = DENSEPriv_GetDisparityVisualization(Disparity16, *StereoSGBM);

    CV_Assert(Disparity16.type() == CV_16SC1);

    const fp64 rep16 = 1.0 / 16.0;

    std::unordered_set<typeVoxelKey, typeVoxelHash> ValidCameraPoints;

    const auto& StartTimeDisparityTo3D = PantoClock::now();

    for(int v = 0; v < Disparity16.rows; v += DENSE_MAP_PIXEL_STRIDE)
    {
        const i16* row = Disparity16.ptr<i16>(v);
        for(int u = 0; u < Disparity16.cols; u += DENSE_MAP_PIXEL_STRIDE)
        {
            const fp64 Disparity = static_cast<fp64>(row[u]) * rep16;

            if(Disparity <= 0.0)
            {
                continue;
            }

            const fp64 Z = fxtimesB / Disparity;
            if(!std::isfinite(Z) || Z < DENSE_MAP_MIN_DEPTH || Z > DENSE_MAP_MAX_DEPTH)
            {
                continue;
            }

            const fp64 X = (u - cx) * Z * fxrep;
            const fp64 Y = (v - cy) * Z * fyrep;

            typeVoxelKey Key = DENSE_GetVoxelKey(X, Y, Z);

            ValidCameraPoints.insert(Key);
        }
    }

    Eigen::Matrix<fp32, 3, Eigen::Dynamic> MapPoints;
    MapPoints.resize(3, static_cast<Eigen::Index>(ValidCameraPoints.size()));

    u64 Col = 0;
    for(const typeVoxelKey& Key : ValidCameraPoints)
    {
        MapPoints.col(Col) = Eigen::Vector3f
            {
                (static_cast<fp32>(Key.X) + 0.5f) * static_cast<fp32>(DENSE_VOXEL_SIZE),
                (static_cast<fp32>(Key.Y) + 0.5f) * static_cast<fp32>(DENSE_VOXEL_SIZE),
                (static_cast<fp32>(Key.Z) + 0.5f) * static_cast<fp32>(DENSE_VOXEL_SIZE)
            };
        Col++;
    }


    typeDenseKeyFrameMap DenseMap = 
    {
        .KeyFrameID = DenseData.KeyFrameID,
        .MapPoints = std::move(MapPoints),
        .DisparityColored = std::move(DisparityColored)
    };

    SumDisparityTo3D += std::chrono::duration<fp64>(PantoClock::now() - StartTimeDisparityTo3D).count();

    return DenseMap;
}


static void DENSEPriv_WriteDenseWCS(const typeDenseMapData& DenseData, const typeDenseLocaMapData& LocalMapData, typeDenseVoxelOccupancyMap& VoxelMap)
{
    //This assumes no keyframes are culled!
    const std::vector<typeKeyFrame> LocalMap = LocalMapData.LocalKeyFrames;

    std::size_t NumberOfPoints = 0;

    std::vector<Eigen::Vector3f> NewWCSMapPoints;
    NewWCSMapPoints.reserve(NumberOfPoints);

    const Eigen::Vector3d VectorVoxelCenter = LocalMapData.LocalKeyFrames.front().Camera.Pose.GetCameraCenter();

    typeVoxelKey VoxelOrigin = DENSE_GetVoxelKey(VectorVoxelCenter.x() - DENSE_MAP_MAX_DEPTH,
            VectorVoxelCenter.y() - DENSE_MAP_MAX_DEPTH,
            VectorVoxelCenter.z() - DENSE_MAP_MAX_DEPTH);
    VoxelMap.OriginKey = VoxelOrigin;

    const PantoClock::time_point GridClearStart = PantoClock::now();
    std::fill(
            VoxelMap.RollingOccupancy.begin(),
            VoxelMap.RollingOccupancy.end(),
            0);
    SumRollingGridClear += std::chrono::duration<fp64>(
            PantoClock::now() - GridClearStart).count();

    const PantoClock::time_point ProjectionStart = PantoClock::now();
    for(const typeKeyFrame& KeyFrame: LocalMapData.LocalKeyFrames)
    {
        const auto DenseMapIterator = DenseData.KeyFrameMaps.find(KeyFrame.ID);
        if(DenseMapIterator == DenseData.KeyFrameMaps.end())
        {
            continue;
        }

        const typeCameraPose& Pose = KeyFrame.Camera.Pose;
        const typeDenseKeyFrameMap& DenseMap = DenseMapIterator->second;

        // Assuming Pose is Tcw:
        // p_camera = Rcw * p_world + tcw
        const Eigen::Matrix3f Rwc = Pose.R.transpose().cast<fp32>();
        const Eigen::Vector3f twc = -Rwc * Pose.t.cast<fp32>();

        Eigen::Matrix<fp32, 3, Eigen::Dynamic> WorldPoints = Rwc * DenseMap.MapPoints;


        WorldPoints.colwise() += twc;

        for(Eigen::Index Column = 0; Column < WorldPoints.cols(); ++Column)
        {
            const Eigen::Vector3f& WorldPoint = WorldPoints.col(Column);

            NewWCSMapPoints.emplace_back(WorldPoint);

            const typeVoxelKey Key = DENSE_GetVoxelKey(WorldPoint.x(), WorldPoint.y(), WorldPoint.z());
            const i32 x = Key.X - VoxelOrigin.X;
            const i32 y = Key.Y - VoxelOrigin.Y;
            const i32 z = Key.Z - VoxelOrigin.Z;

            const i32 n = static_cast<i32>(DENSE_VOXELS_PER_SIDE);

            if (x >= 0 && x < n && y >= 0 && y < n && z >= 0 && z < n)
            {
                const std::size_t index =
                    static_cast<std::size_t>(x) +
                    static_cast<std::size_t>(n) * (static_cast<std::size_t>(y) +
                     static_cast<std::size_t>(n) * static_cast<std::size_t>(z));

                ++VoxelMap.RollingOccupancy[index];
            }
        }
    }
    SumWorldProjectionAndOccupancy += std::chrono::duration<fp64>(
            PantoClock::now() - ProjectionStart).count();

    const PantoClock::time_point WCSPointPublishStart = PantoClock::now();
    {
        std::scoped_lock<std::mutex> Lock(WCSMapMutex);
        WCSMapPoints.swap(NewWCSMapPoints);
    }
    SumWCSPointPublish += std::chrono::duration<fp64>(
            PantoClock::now() - WCSPointPublishStart).count();

    VoxelMap.IsInitialized = true;

    const PantoClock::time_point OccupancyPublishStart = PantoClock::now();
    {
        std::scoped_lock<std::mutex> Lock(RollingVoxelOccupancyMapMutex);
        RollingVoxelOccupancyMap = VoxelMap;
    }
    SumOccupancyPublish += std::chrono::duration<fp64>(
            PantoClock::now() - OccupancyPublishStart).count();
}

std::vector<Eigen::Vector3f> DENSE_GetDenseMapPoints()
{
    std::scoped_lock Lock(WCSMapMutex);
    return WCSMapPoints;
}

typeDenseVoxelOccupancyMap DENSE_GetRollingVoxelOccupancyMap()
{
    std::scoped_lock<std::mutex> Lock(RollingVoxelOccupancyMapMutex);
    return RollingVoxelOccupancyMap;
}


static cv::Mat DENSEPriv_GetDisparityVisualization(const cv::Mat& Disparity16,
    const cv::StereoSGBM& StereoSGBM)
{
    CV_Assert(Disparity16.type() == CV_16SC1);

    const fp32 MinDisparity = static_cast<fp32>(StereoSGBM.getMinDisparity());
    const fp32 NumDisparities = static_cast<fp32>(StereoSGBM.getNumDisparities());

    cv::Mat Disparity32;

    Disparity16.convertTo(Disparity32, CV_32FC1, 1.0 / 16.0);

    // Reject invalid and zero disparities.
    const cv::Mat ValidMask = Disparity32 > std::max(0.0F, MinDisparity);

    cv::Mat Normalized32 = (Disparity32 - MinDisparity) * (255.0F / NumDisparities);

    cv::Mat Normalized8;
    Normalized32.convertTo(Normalized8, CV_8UC1);

    Normalized8.setTo(0, ~ValidMask);

    cv::Mat Colored;
    cv::applyColorMap(Normalized8, Colored, cv::COLORMAP_TURBO);

    // Make invalid disparities black instead of dark blue.
    Colored.setTo(cv::Scalar(0, 0, 0), ~ValidMask);

    return Colored;
}

std::vector<Eigen::Vector4f> DENSEPriv_GetDenseMap(const typeDenseMapData& DenseData, const typeGlobalMap& GlobalMap)
{
    std::vector<Eigen::Vector4f> Ret;
    for(const auto& [KeyFrameID, DenseMap] : DenseData.KeyFrameMaps)
    {
        if(GlobalMap.KeyFrames.contains(KeyFrameID))
        {
            const typeCameraPose& Pose =
                GlobalMap.KeyFrames[KeyFrameID].Camera.Pose;
            const Eigen::Index N = DenseMap.MapPoints.cols();

            Eigen::Matrix4f Tcw = Eigen::Matrix4f::Identity();
            Tcw.block<3, 3>(0, 0) = Pose.R.cast<fp32>();
            Tcw.block<3, 1>(0, 3) = Pose.t.cast<fp32>();

            Eigen::Matrix4f Twc = Tcw.inverse();

            Eigen::Matrix<fp32, 4, Eigen::Dynamic> CameraPoints4(4, N);
            CameraPoints4.topRows<3>() = DenseMap.MapPoints;
            CameraPoints4.row(3).setOnes();

            Eigen::Matrix<fp32, 4, Eigen::Dynamic> WorldPoints4 = Twc * CameraPoints4;

            for (Eigen::Index i = 0; i < N; ++i)
            {
                Ret.push_back(WorldPoints4.col(i));
            }
        }
    }
    return Ret;
}

static bool DENSEPriv_WritePLY(const std::vector<Eigen::Vector4f>& Points, const std::string& Path)
{
    std::size_t NumValidPoints = 0;
    for(const Eigen::Vector4f& Point : Points)
    {
        if(Point.head<3>().allFinite())
        {
            NumValidPoints++;
        }
    }

    std::ofstream Output(Path, std::ios::binary | std::ios::trunc);
    if(!Output.is_open())
    {
        LG_Log(LogSeverity::ERROR,
                "[DENSEPriv_WritePLY] Failed to open %s\n",
                Path.c_str());
        return false;
    }

    Output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << NumValidPoints << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "end_header\n";

    for(const Eigen::Vector4f& Point : Points)
    {
        if(!Point.head<3>().allFinite())
        {
            continue;
        }
        const fp32 Coordinates[3]
        {
            Point.x(),
            Point.y(),
            Point.z()
        };
        Output.write(
                reinterpret_cast<const char*>(Coordinates),
                sizeof(Coordinates));
    }

    if(!Output.good())
    {
        LG_Log(LogSeverity::ERROR,
                "[DENSEPriv_WritePLY] Failed while writing %s\n",
                Path.c_str());
        return false;
    }

    LG_Log(LogSeverity::DATA,
            "[DENSEPriv_WritePLY] Wrote %zu points to %s\n",
            NumValidPoints,
            Path.c_str());
    return true;
}


static void DENSEPriv_LogTimingData(void)
{
    if(NumPairsCalculated == 0)
    {
        LG_Log(LogSeverity::DATA,
                "[DENSE MAPPING] No dense pairs were calculated\n");
        return;
    }

    const fp64 NumDense = static_cast<fp64>(NumPairsCalculated);
    const fp64 MeanQueueWait = SumQueueWait / NumDense;
    const fp64 MeanDenseIteration = SumDenseIteration / NumDense;
    const fp64 MeanTotalDense = SumTotalDenseMapCalc / NumDense;
    const fp64 MeanDisparity  = SumRemapAndDisparity / NumDense;
    const fp64 MeanDispTo3D = SumDisparityTo3D       / NumDense;
    const fp64 MeanDisparityVizEnqueue = SumDisparityVizEnqueue / NumDense;
    const fp64 MeanDenseMapStore = SumDenseMapStore / NumDense;
    const fp64 MeanWriteDenseWCS = SumWriteDenseWCS / NumDense;
    const fp64 MeanRollingGridClear = SumRollingGridClear / NumDense;
    const fp64 MeanWorldProjectionAndOccupancy =
        SumWorldProjectionAndOccupancy / NumDense;
    const fp64 MeanWCSPointPublish = SumWCSPointPublish / NumDense;
    const fp64 MeanOccupancyPublish = SumOccupancyPublish / NumDense;

    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Queue Wait                = %lf\n", MeanQueueWait);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Complete Iteration        = %lf\n", MeanDenseIteration);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Total Dense Mapping = %lf\n", MeanTotalDense);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Remap And Disparity = %lf\n", MeanDisparity);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Disparity To 3D     = %lf\n", MeanDispTo3D);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Disparity Viz Enqueue     = %lf\n", MeanDisparityVizEnqueue);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Dense Map Store           = %lf\n", MeanDenseMapStore);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean WCS/Occupancy Rebuild     = %lf\n", MeanWriteDenseWCS);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Rolling Grid Clear        = %lf\n", MeanRollingGridClear);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Projection And Occupancy  = %lf\n", MeanWorldProjectionAndOccupancy);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean WCS Point Publish         = %lf\n", MeanWCSPointPublish);
    LG_Log(LogSeverity::DATA, "[DENSE MAPPING] Mean Occupancy Publish         = %lf\n", MeanOccupancyPublish);
}

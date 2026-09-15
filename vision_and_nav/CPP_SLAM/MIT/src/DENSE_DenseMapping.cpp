#include "DENSE_DenseMapping.hpp"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "MAP_Mapping.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <cmath>
#include <fstream>
#include <mutex>

std::vector<Eigen::Vector4d> DENSEPriv_GetDenseMap(const typeDenseMapData& DenseData, const typeGlobalMap& GlobalMap);
typeDenseKeyFrameMap DENSEPriv_CalculateDenseKeyFrameMap(const typeDenseData& DenseData);
static cv::Ptr<cv::StereoSGBM> DENSEPriv_InitSGBM(void);
void DENSEPriv_DisplayUpdate(const cv::Mat& Disparity);
static cv::Mat DENSEPriv_GetDisparityVisualization(const cv::Mat& Disparity16,
    const cv::StereoSGBM& StereoSGBM);
static bool DENSEPriv_WritePLY(
        const std::vector<Eigen::Vector4d>& Points,
        const std::string& Path);


void DENSE_DenseMapping(typeDenseMapData& MapData)
{
    typeDenseData DenseData;
    while(true)
    {
        bool DequeRet = MapData.DenseQueue->deque(DenseData);
        // False means stop is signalled.
        if(!DequeRet)
        {
            break;
        }
        typeDenseKeyFrameMap DenseMap = DENSEPriv_CalculateDenseKeyFrameMap(DenseData);
        MapData.VizQueue->enque(DenseMap.DisparityColored);

        MapData.KeyFrameMaps.push_back(std::move(DenseMap));
    }
    std::vector<Eigen::Vector4d> DensePoints;
    {
        std::scoped_lock<std::mutex> Lock(MapData.GlobalMap->Mutex);
        DensePoints = DENSEPriv_GetDenseMap(MapData, *MapData.GlobalMap);
    }
    DENSEPriv_WritePLY(DensePoints, "./PLY_VIZ/dense_map.ply");
    MapData.VizQueue->stop();
}

std::vector<Eigen::Vector4d> DENSEPriv_GetDenseMap(const typeDenseMapData& DenseData, const typeGlobalMap& GlobalMap)
{
    std::vector<Eigen::Vector4d> Ret;
    for(const typeDenseKeyFrameMap& DenseMap : DenseData.KeyFrameMaps)
    {
        const u64 KFID = DenseMap.KeyFrameID;
        if(GlobalMap.KeyFrames.contains(KFID))
        {
            const typeCameraPose& Pose = GlobalMap.KeyFrames[KFID].Camera.Pose;
            const Eigen::Index N = DenseMap.CameraPoints.cols();


            Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
            Tcw.block<3, 3>(0, 0) = Pose.R;
            Tcw.block<3, 1>(0, 3) = Pose.t;

            Eigen::Matrix4d Twc = Tcw.inverse();

            Eigen::Matrix<fp64, 4, Eigen::Dynamic> CameraPoints4(4, N);
            CameraPoints4.topRows<3>() = DenseMap.CameraPoints;
            CameraPoints4.row(3).setOnes();

            Eigen::Matrix<fp64, 4, Eigen::Dynamic> WorldPoints4 = Twc * CameraPoints4;

            for (Eigen::Index i = 0; i < N; ++i)
            {
                Ret.push_back(WorldPoints4.col(i));
            }
        }
    }
    return Ret;
}

static bool DENSEPriv_WritePLY(const std::vector<Eigen::Vector4d>& Points, const std::string& Path)
{
    std::size_t NumValidPoints = 0;
    for(const Eigen::Vector4d& Point : Points)
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

    for(const Eigen::Vector4d& Point : Points)
    {
        if(!Point.head<3>().allFinite())
        {
            continue;
        }
        const fp32 Coordinates[3]
        {
            static_cast<fp32>(Point.x()),
            static_cast<fp32>(Point.y()),
            static_cast<fp32>(Point.z())
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

static cv::Mat DENSEPriv_GetDisparityVisualization(const cv::Mat& Disparity16,
    const cv::StereoSGBM& StereoSGBM)
{
    CV_Assert(Disparity16.type() == CV_16SC1);

    const fp32 MinDisparity = static_cast<fp32>(StereoSGBM.getMinDisparity());

    const fp32 NumDisparities = static_cast<fp32>(StereoSGBM.getNumDisparities());

    cv::Mat Disparity32;
    Disparity16.convertTo(
        Disparity32, CV_32FC1, 1.0 / 16.0);

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

    cv::remap(Left, RectifiedLeft, StereoCalib.Map0X, StereoCalib.Map0Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(Right, RectifiedRight, StereoCalib.Map1X, StereoCalib.Map1Y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat Disparity16;

    StereoSGBM->compute(RectifiedLeft, RectifiedRight, Disparity16);

    cv::Mat DisparityColored = DENSEPriv_GetDisparityVisualization(Disparity16, *StereoSGBM);

    CV_Assert(Disparity16.type() == CV_16SC1);

    const fp64 rep16 = 1.0 / 16.0;
    std::vector<Eigen::Vector3d> ValidCameraPoints;
    ValidCameraPoints.reserve(
            static_cast<std::size_t>(
                (Disparity16.rows / DENSE_MAP_PIXEL_STRIDE + 1) *
                (Disparity16.cols / DENSE_MAP_PIXEL_STRIDE + 1)));

    for(int v = 0; v < Disparity16.rows; v += DENSE_MAP_PIXEL_STRIDE)
    {
        const i16* row = Disparity16.ptr<i16>(v);
        for(int u = 0; u < Disparity16.cols; u += DENSE_MAP_PIXEL_STRIDE)
        {
            const fp64 Disparity = static_cast<fp64>(row[u]) * rep16;

            // Might be more efficient to filter the disparities first, then this can probably be vectorized easier
            if(Disparity <= 0.0)
            {
                continue;
            }

            const fp64 Z = fxtimesB / Disparity;
            if(!std::isfinite(Z) ||
               Z < DENSE_MAP_MIN_DEPTH ||
               Z > DENSE_MAP_MAX_DEPTH)
            {
                continue;
            }

            const fp64 X = (u - cx) * Z * fxrep;
            const fp64 Y = (v - cy) * Z * fyrep;
            ValidCameraPoints.emplace_back(X, Y, Z);
        }
    }

    Eigen::Matrix<fp64, 3, Eigen::Dynamic> MapPoints(
            3, static_cast<Eigen::Index>(ValidCameraPoints.size()));
    for(std::size_t i = 0; i < ValidCameraPoints.size(); i++)
    {
        MapPoints.col(static_cast<Eigen::Index>(i)) = ValidCameraPoints[i];
    }

    typeDenseKeyFrameMap DenseMap = 
    {
        .KeyFrameID = DenseData.KeyFrameID,
        .CameraPoints = std::move(MapPoints),
        .DisparityColored = DisparityColored
    };

    return DenseMap;
}

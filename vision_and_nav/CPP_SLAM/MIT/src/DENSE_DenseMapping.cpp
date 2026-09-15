#include "DENSE_DenseMapping.hpp"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "opencv2/core.hpp"

static cv::Ptr<cv::StereoSGBM> DENSEPriv_InitSGBM(void);

typeDenseKeyFrameMap DENSE_CalculateDenseKeyFrameMap(const typeDenseData& DenseData)
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

    Eigen::Matrix<fp64, Eigen::Dynamic, 3> MapPoints;
    MapPoints.resize(Disparity16.rows * Disparity16.cols, 3);
    
    CV_Assert(Disparity16.type() == CV_16SC1);

    for(int v = 0; v < Disparity16.rows; v++)
    {
        const i16* row = Disparity16.ptr<i16>(v);
        for(int u = 0; u < Disparity16.cols; u++)
        {
            const fp64 Disparity = static_cast<fp64>(row[u]);

            // Might be more efficient to filter the disparities first, then this can probably be vectorized easier
            if(Disparity < 0.0)
            {
                continue;
            }

            const i32 i = v * Disparity16.cols + u;

            const fp64 Z = fxtimesB / Disparity;
            const fp64 X = (u - cx) * Z * fxrep;
            const fp64 Y = (v - cy) * Z * fyrep;
            MapPoints.row(i) << X , Y , Z;
        }
    }

    typeDenseKeyFrameMap DenseMap = 
    {
        .KeyFrameID = DenseData.KeyFrameID,
        .CameraPoints = MapPoints.transpose()
    };

    return DenseMap;
}

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
        MapData.KeyFrameMaps.push_back(DENSE_CalculateDenseKeyFrameMap(DenseData));
    }
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

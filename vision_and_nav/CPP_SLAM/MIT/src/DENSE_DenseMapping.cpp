#include "DENSE_DenseMapping.hpp"
#include "opencv2/core/mat.hpp"

typeDenseKeyFrameMap DENSE_CalculateDenseKeyFrameMap(const u64 KeyFrameID, const typePantoFrame& Frame)
{
    const cv::Mat& Left = Frame.Frame;
    const cv::Mat& Right = Frame.RightFrame;

}

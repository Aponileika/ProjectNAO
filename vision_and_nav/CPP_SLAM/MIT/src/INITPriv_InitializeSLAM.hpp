#ifndef INITPRIV_INITIALIZESLAM_HPP_
#define INITPRIV_INITIALIZESLAM_HPP_
#include "../include/INIT_InitializeSLAM.hpp"

class ImageToImageMapping
{
public:
    virtual fp64 Error(
            const Eigen::Vector2d& Point1,
            const Eigen::Vector2d& Point2) const = 0;

    virtual ~ImageToImageMapping() = default;
};

class FundamentalMatrixMapping : public ImageToImageMapping
{
public:
    static constexpr u64 MinimumPoints = 8;

    using Model = Eigen::Matrix3d;

    Model FundamentalMatrix;

    fp64 MaxScore;

    static Model Estimate( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        return F_EstimateMinimal(Points1, Points2);
    }

    fp64 Error(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2) const override
    {
        return F_GetError( Point1, Point2);
    }

private:
    static Model F_EstimateMinimal(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        //8-point algorithm
    }

    static fp64 F_GetError(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2)
    {
        // symmetric Epipolar line distance
    }
};

class HomographyMapping : public ImageToImageMapping
{
public:
    static constexpr u64 MinimumPoints = 4;

    using Model = Eigen::Matrix3d;

    Model Homography;

    fp64 MaxScore;

    static Model Estimate(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        return H_EstimateMinimal(Points1, Points2);
    }

    fp64 Error(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2) const override
    {
        return H_GetError(Point1, Point2);
    }

private:
    static Model H_EstimateMinimal(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        //4-point algorithm
    }

    static fp64 H_GetError(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2)
    {
        // Symmetric transfer error (projection)
    }
};

void INITPriv_MatchHistoricalFrames(void);
void INITPriv_STRANSAC(void);
std::unique_ptr<ImageToImageMapping> INITPriv_ScoredFAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical);
bool INITPriv_EnoughStationaryFeatures(void);
void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp);

#endif //  INITPRIV_INITIALIZESLAM_HPP_

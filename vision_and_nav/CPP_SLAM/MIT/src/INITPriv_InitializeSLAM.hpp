#ifndef INITPRIV_INITIALIZESLAM_HPP_
#define INITPRIV_INITIALIZESLAM_HPP_
#include "../include/INIT_InitializeSLAM.hpp"

class FundamentalMatrixMapping 
{
public:
    static constexpr u64 MinimumPoints = 8;

    using Model = Eigen::Matrix3d;

    static Model Estimate( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        return F_Estimate( Points1, Points2);
    }

    static fp64 Error( const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2, const Model& F)
    {
        return F_GetError( Point1, Point2, F);
    }

private:
    static Model F_Estimate( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        //8-point algorithm
    }

    static fp64 F_GetError( const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2, const Model& F)
    {
        // symmetric Epipolar line distance
    }
};

class HomographyMapping 
{
public:
    static constexpr u64 MinimumPoints = 4;

        using Model = Eigen::Matrix3d;

    static Model Estimate(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        return H_Estimate(Points1, Points2);
    }

    static fp64 Error(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2, const Model& H)
    {
        return H_GetError(Point1, Point2, H);
    }

private:
    static Model H_Estimate( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2)
    {
        //4-point algorithm
    }

    static fp64 H_GetError(const Eigen::Vector2d& Point1, const Eigen::Vector2d& Point2, const Model& H)
    {
        // Symmetric transfer error (projection)
    }
};

void INITPriv_MatchHistoricalFrames(void);

template<typename Mapping>
void INITPriv_STRANSAC(void);

bool INITPriv_EnoughStationaryFeatures(void);
void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp);

#endif //  INITPRIV_INITIALIZESLAM_HPP_

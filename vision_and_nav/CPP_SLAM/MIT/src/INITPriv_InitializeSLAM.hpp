#ifndef INITPRIV_INITIALIZESLAM_HPP_
#define INITPRIV_INITIALIZESLAM_HPP_
#include "../include/INIT_InitializeSLAM.hpp"
#include <cmath>
#include <random>

class ImageToImageMapping
{
public:
    virtual std::vector<fp64> Error(const std::vector<Eigen::Vector2d>& Point1, const std::vector<Eigen::Vector2d>& Point2) const = 0; 

    virtual fp64 GetErrorThreshold() const = 0;

    virtual ~ImageToImageMapping() = default;
};

class FundamentalMatrixMapping : public ImageToImageMapping
{
public:
    using Model = Eigen::Matrix3d;

    Model FundamentalMatrix;

    fp64 MaxScore = 0.0;

    const fp64 ErrorThreshold = PANTO_INIT_ERROR_THRESHOLD_INLIER_FUNDAMENTAL;

    void Estimate(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2, const u64 Seed)
    {
        std::mt19937_64 Generator(Seed);

        const std::size_t NumPoints = Points1.size();

        std::vector<u64> PointIndexes(NumPoints);

        std::iota(PointIndexes.begin(), PointIndexes.end(), 0);

        for(u64 i = 0; i < PANTO_INIT_RANSAC_LOOP_CNT; i++)
        {
            Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS> PointsMinimal1;
            Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS> PointsMinimal2;

            for(u64 j = 0; j < PANTO_FUNDAMENTAL_MIN_POINTS; ++j)
            {

                std::uniform_int_distribution<u64> Distribution(j, NumPoints - 1);

                const u64 RandomIndex = Distribution(Generator);

                std::swap(PointIndexes[j], PointIndexes[RandomIndex]);

                const u64 RandomSampleIndex = PointIndexes[i];

                const Eigen::Vector2d& Point1 = Points1[RandomSampleIndex];
                const Eigen::Vector2d& Point2 = Points2[RandomSampleIndex];

                PointsMinimal1.col(i) << Point1.x(), Point1.y(), 1.0;
                PointsMinimal2.col(i) << Point2.x(), Point2.y(), 1.0;
            }

            Model FCurrent = F_EstimateMinimal(PointsMinimal1, PointsMinimal2);

            std::vector<fp64> Errors = F_Error(Points1, Points2, FCurrent);

            fp64 Score = 0;

            for(const fp64 Error : Errors)
            {
                if(Error < PANTO_INIT_ERROR_THRESHOLD_INLIER_FUNDAMENTAL)
                {
                    Score += 1.0 - Error / ErrorThreshold;
                }
            }
            if(Score > MaxScore)
            {
                MaxScore = Score;
                FundamentalMatrix = FCurrent;
            }
        }
    }

    std::vector<fp64> Error(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2) const override
    {
        assert(Points1.size() == Points2.size());

        std::vector<fp64> Errors(Points1.size());

        const Model& F = FundamentalMatrix;

        for(std::size_t i = 0; i < Points1.size(); ++i)
        {
            const Eigen::Vector3d Point1( Points1[i].x(), Points1[i].y(), 1.0);

            const Eigen::Vector3d Point2( Points2[i].x(), Points2[i].y(), 1.0);

            const Eigen::Vector3d Line2 = F * Point1;
            const Eigen::Vector3d Line1 = F.transpose() * Point2;

            const fp64 Residual = Point2.dot(Line2);
            const fp64 ResidualSquared = Residual * Residual;

            const fp64 DistanceSquared1 = ResidualSquared / (Line1.x() * Line1.x() + Line1.y() * Line1.y());
            const fp64 DistanceSquared2 = ResidualSquared / (Line2.x() * Line2.x() + Line2.y() * Line2.y());

            Errors[i] = std::max(DistanceSquared1, DistanceSquared2);
        }

        return Errors;
    }

    fp64 GetErrorThreshold(void) const override
    {
        return ErrorThreshold;
    }

private:
    static Model F_EstimateMinimal(const Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS>& Points1, const Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS>& Points2)
    {
        //8-point algorithm
        Eigen::Matrix<fp64, 8, 9> A;

        for(u64 i = 0; i < 8; ++i)
        {
            const fp64 x1 = Points1(0, i);
            const fp64 y1 = Points1(1, i);
            const fp64 x2 = Points2(0, i);
            const fp64 y2 = Points2(1, i);

            A.row(i) << x2 * x1,
                x2 * y1, 
                x2, 
                y2 * x1, 
                y2 * y1, 
                y2, 
                x1, 
                y1, 
                1.0; 
        }

        Eigen::JacobiSVD<Eigen::Matrix<fp64, 8, 9>> SVD(A, Eigen::ComputeFullV);

        const Eigen::Matrix<fp64, 9, 1> FVector = SVD.matrixV().col(8);

        Model F = Eigen::Map< const Eigen::Matrix<fp64, 3, 3, Eigen::RowMajor>>(FVector.data());

        return F;
    }

    std::vector<fp64> F_Error( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2, const Model& F)
    {
        assert(Points1.size() == Points2.size());

        std::vector<fp64> Errors(Points1.size());

        for(std::size_t i = 0; i < Points1.size(); ++i)
        {
            const Eigen::Vector3d Point1( Points1[i].x(), Points1[i].y(), 1.0);

            const Eigen::Vector3d Point2( Points2[i].x(), Points2[i].y(), 1.0);

            const Eigen::Vector3d Line2 = F * Point1;
            const Eigen::Vector3d Line1 = F.transpose() * Point2;

            const fp64 Residual = Point2.dot(Line2);
            const fp64 ResidualSquared = Residual * Residual;

            const fp64 DistanceSquared1 = ResidualSquared / (Line1.x() * Line1.x() + Line1.y() * Line1.y());
            const fp64 DistanceSquared2 = ResidualSquared / (Line2.x() * Line2.x() + Line2.y() * Line2.y());

            Errors[i] = std::max(DistanceSquared1, DistanceSquared2);
        }

        return Errors;
    }

};

class HomographyMapping : public ImageToImageMapping
{
public:
    static constexpr u64 MinimumPoints = PANTO_HOMOGRAPHY_MIN_POINTS;

    using Model = Eigen::Matrix3d;

    Model Homography;

    fp64 MaxScore = 0.0;

    Model Estimate(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2, const u64 Seed)
    {
        std::mt19937_64 Generator(Seed);

        const std::size_t NumPoints = Points1.size();

        std::vector<u64> PointIndexes(NumPoints);

        std::iota(PointIndexes.begin(), PointIndexes.end(), 0);

        for(u64 i = 0; i < PANTO_INIT_RANSAC_LOOP_CNT; i++)
        {
            Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS> PointsMinimal1;
            Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS> PointsMinimal2;

            for(u64 j = 0; j < PANTO_FUNDAMENTAL_MIN_POINTS; ++j)
            {
                
                std::uniform_int_distribution<u64> Distribution(j, NumPoints - 1);

                const u64 RandomIndex = Distribution(Generator);

                std::swap(PointIndexes[j], PointIndexes[RandomIndex]);

                const u64 RandomSampleIndex = PointIndexes[i];

                if(j < PANTO_HOMOGRAPHY_MIN_POINTS)
                {
                    const Eigen::Vector2d& Point1 = Points1[RandomSampleIndex];
                    const Eigen::Vector2d& Point2 = Points2[RandomSampleIndex];

                    PointsMinimal1.col(i) << Point1.x(), Point1.y(), 1.0;
                    PointsMinimal2.col(i) << Point2.x(), Point2.y(), 1.0;
                }

            }

            Model HCurrent = H_EstimateMinimal(PointsMinimal1, PointsMinimal2);

            std::vector<fp64> Error = H_Error(Points1, Points2, HCurrent);
        }

    }

    std::vector<fp64> Error(const std::vector<Eigen::Vector2d>& Point1, const std::vector<Eigen::Vector2d>& Point2) const override
    {
        // remember to divide by the error threshold
        return std::vector<fp64>{};
    }

private:

    static Model H_EstimateMinimal(const Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS>& Points1, const Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS>& Points2)
    {
        //4-point algorithm
    }

    std::vector<fp64> H_Error(const std::vector<Eigen::Vector2d>& Point1, const std::vector<Eigen::Vector2d>& Point2, const Model& H) 
    {
    }
};

void INITPriv_MatchHistoricalFrames(void);
std::vector<typeInitImagePoint> INITPriv_STRANSAC(void);
u64 INITPriv_RandomSeed(void);
std::unique_ptr<ImageToImageMapping> INITPriv_ScoredFAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical);
bool INITPriv_EnoughStationaryFeatures(void);
void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp);

#endif //  INITPRIV_INITIALIZESLAM_HPP_

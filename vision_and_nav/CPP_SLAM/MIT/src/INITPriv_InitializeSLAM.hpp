#ifndef INITPRIV_INITIALIZESLAM_HPP_
#define INITPRIV_INITIALIZESLAM_HPP_
#include "../include/INIT_InitializeSLAM.hpp"
#include "PT_PantoMapPoints.hpp"
#include <cmath>
#include "Config.hpp"
#include <random>

template <int N>
static void INITPriv_NormalizePoints2D( const Eigen::Matrix<fp64, 3, N>& Points, Eigen::Matrix<fp64, 3, N>& PointsNormalized, Eigen::Matrix3d& T)
{
    fp64 MeanX = 0.0;
    fp64 MeanY = 0.0;

    for(i32 i = 0; i < N; ++i)
    {
        const fp64 InvZ = 1.0 / Points(2, i);
        MeanX += Points(0, i) * InvZ;
        MeanY += Points(1, i) * InvZ;
    }

    MeanX /= static_cast<fp64>(N);
    MeanY /= static_cast<fp64>(N);

    fp64 MeanDistance = 0.0;

    for(i32 i = 0; i < N; ++i)
    {
        const fp64 InvZ = 1.0 / Points(2, i);
        const fp64 X = Points(0, i) * InvZ;
        const fp64 Y = Points(1, i) * InvZ;

        const fp64 DX = X - MeanX;
        const fp64 DY = Y - MeanY;

        MeanDistance += std::sqrt(DX * DX + DY * DY);
    }

    MeanDistance /= static_cast<fp64>(N);

    fp64 Scale = 1.0;
    if(MeanDistance > std::numeric_limits<fp64>::epsilon())
    {
        Scale = std::sqrt(2.0) / MeanDistance;
    }

    T <<
        Scale, 0.0,   -Scale * MeanX,
        0.0,   Scale, -Scale * MeanY,
        0.0,   0.0,    1.0;

    PointsNormalized = T * Points;
}

class ImageToImageMapping
{
public:
    virtual std::vector<fp64> Error(const std::vector<Eigen::Vector2d>& Point1, const std::vector<Eigen::Vector2d>& Point2) const = 0; 

    virtual fp64 GetErrorThreshold() const = 0;

    virtual typeInitReconstruction Reconstruct( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2,
        const std::vector<std::pair<u64, u64>>& ImagePointIDs, const std::pair<u64, u64>& InitFrameIDs, const Eigen::Matrix3d& K) const = 0;

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
                const u64 RandomSampleIndex = PointIndexes[j];

                const Eigen::Vector2d& Point1 = Points1[RandomSampleIndex];
                const Eigen::Vector2d& Point2 = Points2[RandomSampleIndex];

                PointsMinimal1.col(j) << Point1.x(), Point1.y(), 1.0;
                PointsMinimal2.col(j) << Point2.x(), Point2.y(), 1.0;
            }

            Model FCurrent = F_EstimateMinimal(PointsMinimal1, PointsMinimal2);

            std::vector<fp64> Errors = F_Error(Points1, Points2, FCurrent);

            fp64 Score = 0;

            for(const fp64 Error : Errors)
            {
                if(Error < ErrorThreshold)
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

    typeInitReconstruction Reconstruct( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2,
    const std::vector<std::pair<u64, u64>>& ImagePointIDs, const std::pair<u64, u64>& InitFrameIDs, const Eigen::Matrix3d& K) const override
    {
        assert(Points1.size() == Points2.size());
        assert(Points1.size() == ImagePointIDs.size());

        typeInitReconstruction BestReconstruction{};
        BestReconstruction.NumPointsInFront = 0;
        BestReconstruction.ChosenInitFrameID = InitFrameIDs;
        BestReconstruction.Valid = false;

        /*
         * x2^T F x1 = 0
         *
         * E = K^T F K
         */
        const Eigen::Matrix3d E =
            K.transpose() * FundamentalMatrix * K;

        Eigen::JacobiSVD<Eigen::Matrix3d> SVD(
            E,
            Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3d U = SVD.matrixU();
        Eigen::Matrix3d V = SVD.matrixV();

        if(U.determinant() < 0.0)
        {
            U.col(2) *= -1.0;
        }

        if(V.determinant() < 0.0)
        {
            V.col(2) *= -1.0;
        }

        Eigen::Matrix3d W;

        W <<
             0.0, -1.0, 0.0,
             1.0,  0.0, 0.0,
             0.0,  0.0, 1.0;

        Eigen::Matrix3d R1 =
            U * W * V.transpose();

        Eigen::Matrix3d R2 =
            U * W.transpose() * V.transpose();

        if(R1.determinant() < 0.0)
        {
            R1 = -R1;
        }

        if(R2.determinant() < 0.0)
        {
            R2 = -R2;
        }

        Eigen::Vector3d t = U.col(2);
        t.normalize();

        const std::array<Eigen::Matrix3d, 4> Rotations = { R1, R1, R2, R2 };

        const std::array<Eigen::Vector3d, 4> Translations = { t, -t, t, -t };

        Eigen::Matrix<fp64, 3, 4> Rt1 = Eigen::Matrix<fp64, 3, 4>::Zero();

        Rt1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        /*
         * Undistorted PIXEL coordinates:
         *
         * P1 = K [I | 0]
         */
        const Eigen::Matrix<fp64, 3, 4> P1 =
            K * Rt1;

        for(u64 HypothesisID = 0;
            HypothesisID < 4;
            ++HypothesisID)
        {
            const Eigen::Matrix3d& R =
                Rotations[HypothesisID];

            const Eigen::Vector3d& Translation =
                Translations[HypothesisID];

            const Eigen::Vector3d CameraCenter1 = Eigen::Vector3d::Zero();
            const Eigen::Vector3d CameraCenter2 =
                -R.transpose() * Translation;

            std::vector<fp64> CosParallaxes;
            CosParallaxes.reserve(Points1.size());

            Eigen::Matrix<fp64, 3, 4> Rt2;

            Rt2.block<3, 3>(0, 0) = R;
            Rt2.col(3) = Translation;

            const Eigen::Matrix<fp64, 3, 4> P2 =
                K * Rt2;

            std::vector<typeInitMapPoint> MapPoints;
            MapPoints.reserve(Points1.size());

            u64 NumPointsInFront = 0;

            for(std::size_t i = 0;
                i < Points1.size();
                ++i)
            {
                const Eigen::Vector4d Point4D =
                    PROJ_TriangulateDLT(
                        Points1[i],
                        Points2[i],
                        P1,
                        P2);

                if(!Point4D.allFinite())
                {
                    continue;
                }

                const Eigen::Vector3d PointCamera1 =
                    Point4D.head<3>();

                if(PointCamera1.z() <= 0.0)
                {
                    continue;
                }

                const Eigen::Vector3d PointCamera2 =
                    R * PointCamera1 + Translation;

                if(PointCamera2.z() <= 0.0)
                {
                    continue;
                }

                const Eigen::Vector3d Projected1 =
                    K * PointCamera1;

                const Eigen::Vector2d ReprojectedPoint1 =
                {
                    Projected1.x() / Projected1.z(),
                    Projected1.y() / Projected1.z()
                };

                const Eigen::Vector3d Projected2 =
                    K * PointCamera2;

                const Eigen::Vector2d ReprojectedPoint2 =
                {
                    Projected2.x() / Projected2.z(),
                    Projected2.y() / Projected2.z()
                };

                const fp64 ReprojectionError1 =
                    (ReprojectedPoint1 - Points1[i]).squaredNorm();

                const fp64 ReprojectionError2 =
                    (ReprojectedPoint2 - Points2[i]).squaredNorm();

                if(ReprojectionError1 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED ||
                   ReprojectionError2 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED)
                {
                    continue;
                }

                const Eigen::Vector3d Ray1 =
                    PointCamera1;

                const Eigen::Vector3d Ray2 =
                    PointCamera1 - CameraCenter2;

                const fp64 CosParallax =
                    Ray1.dot(Ray2) /
                    (Ray1.norm() * Ray2.norm());

                if(!std::isfinite(CosParallax))
                {
                    continue;
                }

                CosParallaxes.push_back(
                    std::clamp(CosParallax, -1.0, 1.0));

                MapPoints.push_back(
                {
                    .Point4D = Point4D,
                    .InitImagePointID = ImagePointIDs[i]
                });

                ++NumPointsInFront;
            }

            if(CosParallaxes.empty())
            {
                continue;
            }

            std::sort(
                    CosParallaxes.begin(),
                    CosParallaxes.end());

            const std::size_t ParallaxIndex =
                std::min<std::size_t>(
                        50,
                        CosParallaxes.size() - 1);

            const fp64 Parallax =
                std::acos(
                        CosParallaxes[ParallaxIndex]) *
                180.0 / M_PI;

            if(Parallax < PANTO_INIT_MIN_PARALLAX_DEGREES)
            {
                continue;
            }

            if(NumPointsInFront >
               BestReconstruction.NumPointsInFront)
            {
                BestReconstruction.R = R;
                BestReconstruction.t = Translation;

                BestReconstruction.MapPoints =
                    std::move(MapPoints);

                BestReconstruction.NumPointsInFront =
                    NumPointsInFront;

                BestReconstruction.Valid = true;
            }
        }

        return BestReconstruction;
    }


private:
    static Model F_EstimateMinimal( const Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS>& Points1, const Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS>& Points2)
    {
        Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS> PointsNormalized1;
        Eigen::Matrix<fp64, 3, PANTO_FUNDAMENTAL_MIN_POINTS> PointsNormalized2;

        Eigen::Matrix3d T1;
        Eigen::Matrix3d T2;

        INITPriv_NormalizePoints2D<PANTO_FUNDAMENTAL_MIN_POINTS>( Points1, PointsNormalized1, T1);

        INITPriv_NormalizePoints2D<PANTO_FUNDAMENTAL_MIN_POINTS>( Points2, PointsNormalized2, T2);

        Eigen::Matrix<fp64, 8, 9> A;

        for(i32 i = 0; i < PANTO_FUNDAMENTAL_MIN_POINTS; ++i)
        {
            const fp64 X1 = PointsNormalized1(0, i);
            const fp64 Y1 = PointsNormalized1(1, i);
            const fp64 X2 = PointsNormalized2(0, i);
            const fp64 Y2 = PointsNormalized2(1, i);

            A.row(i) << X2 * X1,
                X2 * Y1,
                X2,
                Y2 * X1,
                Y2 * Y1,
                Y2,
                X1,
                Y1,
                1.0;
        }

        Eigen::JacobiSVD<Eigen::Matrix<fp64, 8, 9>> SVD_A( A, Eigen::ComputeFullV);

        const Eigen::Matrix<fp64, 9, 1> FVector = SVD_A.matrixV().col(8);

        Eigen::Matrix3d FNormalized = Eigen::Map<const Eigen::Matrix<fp64, 3, 3, Eigen::RowMajor>>(FVector.data());

        Eigen::JacobiSVD<Eigen::Matrix3d> SVD_F(FNormalized, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector3d SingularValues = SVD_F.singularValues();

        Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
        Sigma(0, 0) = SingularValues(0);
        Sigma(1, 1) = SingularValues(1);
        Sigma(2, 2) = 0.0;

        FNormalized = SVD_F.matrixU() * Sigma * SVD_F.matrixV().transpose();

        Model F = T2.transpose() * FNormalized * T1;

        const fp64 FNorm = F.norm();
        if(FNorm > std::numeric_limits<fp64>::epsilon())
        {
            F /= FNorm;
        }

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

    const fp64 ErrorThreshold = PANTO_INIT_ERROR_THRESHOLD_INLIER_HOMOGRAPHY;

    void Estimate(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2, const u64 Seed)
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

                const u64 RandomSampleIndex = PointIndexes[j];

                if(j < PANTO_HOMOGRAPHY_MIN_POINTS)
                {
                    const Eigen::Vector2d& Point1 = Points1[RandomSampleIndex];
                    const Eigen::Vector2d& Point2 = Points2[RandomSampleIndex];

                    PointsMinimal1.col(j) << Point1.x(), Point1.y(), 1.0;
                    PointsMinimal2.col(j) << Point2.x(), Point2.y(), 1.0;
                }

            }

            Model HCurrent = H_EstimateMinimal(PointsMinimal1, PointsMinimal2);

            std::vector<fp64> Errors = H_Error(Points1, Points2, HCurrent);

            fp64 Score = 0;

            for(const fp64 Error : Errors)
            {
                if(Error < ErrorThreshold)
                {
                    Score += 1.0 - Error / ErrorThreshold;
                }
            }
            if(Score > MaxScore)
            {
                MaxScore = Score;
                Homography = HCurrent;
            }
        }
    }

    std::vector<fp64> Error( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2) const override
    {
        assert(Points1.size() == Points2.size());

        std::vector<fp64> Errors(Points1.size());

        const Model& H = Homography;
        const Model HInverse = H.inverse();

        for(std::size_t i = 0; i < Points1.size(); ++i)
        {
            const Eigen::Vector3d Point1( Points1[i].x(), Points1[i].y(), 1.0);
            const Eigen::Vector3d Point2( Points2[i].x(), Points2[i].y(), 1.0);

            const Eigen::Vector3d ProjectedPoint2Homogeneous = H * Point1;
            const Eigen::Vector3d ProjectedPoint1Homogeneous = HInverse * Point2;

            const Eigen::Vector2d ProjectedPoint2( ProjectedPoint2Homogeneous.x() / ProjectedPoint2Homogeneous.z(),
                ProjectedPoint2Homogeneous.y() / ProjectedPoint2Homogeneous.z());

            const Eigen::Vector2d ProjectedPoint1( ProjectedPoint1Homogeneous.x() / ProjectedPoint1Homogeneous.z(),
                ProjectedPoint1Homogeneous.y() / ProjectedPoint1Homogeneous.z());

            const fp64 DistanceSquared12 = (Points2[i] - ProjectedPoint2).squaredNorm();

            const fp64 DistanceSquared21 = (Points1[i] - ProjectedPoint1).squaredNorm();

            Errors[i] = std::max(DistanceSquared12, DistanceSquared21);
        }
        return Errors;
    }

    fp64 GetErrorThreshold(void) const override
    {
        return ErrorThreshold;
    }

    typeInitReconstruction Reconstruct( const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2,
    const std::vector<std::pair<u64, u64>>& ImagePointIDs, const std::pair<u64, u64>& InitFrameIDs, const Eigen::Matrix3d& K) const override
    {
        assert(Points1.size() == Points2.size());
        assert(Points1.size() == ImagePointIDs.size());

        typeInitReconstruction BestReconstruction{};
        BestReconstruction.NumPointsInFront = 0;
        BestReconstruction.ChosenInitFrameID = InitFrameIDs;
        BestReconstruction.Valid = false;

        const Eigen::Matrix3d A =
            K.inverse() * Homography * K;

        const Eigen::JacobiSVD<Eigen::Matrix3d> SVD(
            A,
            Eigen::ComputeFullU | Eigen::ComputeFullV);

        const Eigen::Matrix3d U = SVD.matrixU();
        const Eigen::Matrix3d V = SVD.matrixV();

        const Eigen::Vector3d SingularValues =
            SVD.singularValues();

        const fp64 d1 = SingularValues(0);
        const fp64 d2 = SingularValues(1);
        const fp64 d3 = SingularValues(2);

        if(d1 / d2 < 1.00001 ||
           d2 / d3 < 1.00001)
        {
            return BestReconstruction;
        }

        const fp64 s =
            U.determinant() * V.determinant();

        std::array<Eigen::Matrix3d, 8> Rotations;
        std::array<Eigen::Vector3d, 8> Translations;

        /*
         * Solutions 0..3
         */
        const fp64 Aux1 =
            std::sqrt(
                (d1 * d1 - d2 * d2) /
                (d1 * d1 - d3 * d3));

        const fp64 Aux3 =
            std::sqrt(
                (d2 * d2 - d3 * d3) /
                (d1 * d1 - d3 * d3));

        const std::array<fp64, 4> X1 =
        {
             Aux1,
             Aux1,
            -Aux1,
            -Aux1
        };

        const std::array<fp64, 4> X3 =
        {
             Aux3,
            -Aux3,
             Aux3,
            -Aux3
        };

        const fp64 AuxSinTheta =
            std::sqrt(
                (d1 * d1 - d2 * d2) *
                (d2 * d2 - d3 * d3)) /
            ((d1 + d3) * d2);

        const fp64 CosTheta =
            (d2 * d2 + d1 * d3) /
            ((d1 + d3) * d2);

        const std::array<fp64, 4> SinTheta =
        {
             AuxSinTheta,
            -AuxSinTheta,
            -AuxSinTheta,
             AuxSinTheta
        };

        for(u64 i = 0; i < 4; ++i)
        {
            Eigen::Matrix3d RPrime =
                Eigen::Matrix3d::Identity();

            RPrime(0, 0) = CosTheta;
            RPrime(0, 2) = -SinTheta[i];
            RPrime(2, 0) = SinTheta[i];
            RPrime(2, 2) = CosTheta;

            Rotations[i] =
                s * U * RPrime * V.transpose();

            Eigen::Vector3d tPrime;

            tPrime <<
                X1[i],
                0.0,
                -X3[i];

            tPrime *= d1 - d3;

            Eigen::Vector3d t =
                U * tPrime;

            if(t.norm() > 0.0)
            {
                t.normalize();
            }

            Translations[i] = t;
        }

        /*
         * Solutions 4..7
         */
        const fp64 AuxSinPhi =
            std::sqrt(
                (d1 * d1 - d2 * d2) *
                (d2 * d2 - d3 * d3)) /
            ((d1 - d3) * d2);

        const fp64 CosPhi =
            (d1 * d3 - d2 * d2) /
            ((d1 - d3) * d2);

        const std::array<fp64, 4> SinPhi =
        {
             AuxSinPhi,
            -AuxSinPhi,
            -AuxSinPhi,
             AuxSinPhi
        };

        for(u64 i = 0; i < 4; ++i)
        {
            Eigen::Matrix3d RPrime =
                Eigen::Matrix3d::Identity();

            RPrime(0, 0) = CosPhi;
            RPrime(0, 2) = SinPhi[i];
            RPrime(1, 1) = -1.0;
            RPrime(2, 0) = SinPhi[i];
            RPrime(2, 2) = -CosPhi;

            Rotations[i + 4] =
                s * U * RPrime * V.transpose();

            Eigen::Vector3d tPrime;

            tPrime <<
                X1[i],
                0.0,
                X3[i];

            tPrime *= d1 + d3;

            Eigen::Vector3d t =
                U * tPrime;

            if(t.norm() > 0.0)
            {
                t.normalize();
            }

            Translations[i + 4] = t;
        }

        Eigen::Matrix<fp64, 3, 4> Rt1 =
            Eigen::Matrix<fp64, 3, 4>::Zero();

        Rt1.block<3, 3>(0, 0) =
            Eigen::Matrix3d::Identity();

        const Eigen::Matrix<fp64, 3, 4> P1 =
            K * Rt1;

        for(u64 HypothesisID = 0;
            HypothesisID < 8;
            ++HypothesisID)
        {
            const Eigen::Matrix3d& R =
                Rotations[HypothesisID];

            const Eigen::Vector3d& Translation =
                Translations[HypothesisID];

            const Eigen::Vector3d CameraCenter1 = Eigen::Vector3d::Zero();
            const Eigen::Vector3d CameraCenter2 =
                -R.transpose() * Translation;

            std::vector<fp64> CosParallaxes;
            CosParallaxes.reserve(Points1.size());

            Eigen::Matrix<fp64, 3, 4> Rt2;

            Rt2.block<3, 3>(0, 0) = R;
            Rt2.col(3) = Translation;

            const Eigen::Matrix<fp64, 3, 4> P2 =
                K * Rt2;

            std::vector<typeInitMapPoint> MapPoints;
            MapPoints.reserve(Points1.size());

            u64 NumPointsInFront = 0;

            for(std::size_t i = 0;
                i < Points1.size();
                ++i)
            {
                const Eigen::Vector4d Point4D =
                    PROJ_TriangulateDLT(
                        Points1[i],
                        Points2[i],
                        P1,
                        P2);

                if(!Point4D.allFinite())
                {
                    continue;
                }

                const Eigen::Vector3d PointCamera1 =
                    Point4D.head<3>();

                if(PointCamera1.z() <= 0.0)
                {
                    continue;
                }

                const Eigen::Vector3d PointCamera2 =
                    R * PointCamera1 + Translation;

                if(PointCamera2.z() <= 0.0)
                {
                    continue;
                }

                const Eigen::Vector3d Projected1 =
                    K * PointCamera1;

                const Eigen::Vector2d ReprojectedPoint1 =
                {
                    Projected1.x() / Projected1.z(),
                    Projected1.y() / Projected1.z()
                };

                const Eigen::Vector3d Projected2 =
                    K * PointCamera2;

                const Eigen::Vector2d ReprojectedPoint2 =
                {
                    Projected2.x() / Projected2.z(),
                    Projected2.y() / Projected2.z()
                };

                const fp64 ReprojectionError1 =
                    (ReprojectedPoint1 - Points1[i]).squaredNorm();

                const fp64 ReprojectionError2 =
                    (ReprojectedPoint2 - Points2[i]).squaredNorm();

                if(ReprojectionError1 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED ||
                   ReprojectionError2 > PANTO_INIT_MAX_REPROJECTION_ERROR_SQUARED)
                {
                    continue;

                }
                const Eigen::Vector3d Ray1 =
                    PointCamera1;

                const Eigen::Vector3d Ray2 =
                    PointCamera1 - CameraCenter2;

                const fp64 CosParallax =
                    Ray1.dot(Ray2) /
                    (Ray1.norm() * Ray2.norm());

                if(!std::isfinite(CosParallax))
                {
                    continue;
                }

                CosParallaxes.push_back(
                        std::clamp(CosParallax, -1.0, 1.0));

                MapPoints.push_back(
                        {
                        .Point4D = Point4D,
                        .InitImagePointID = ImagePointIDs[i]
                        });

                ++NumPointsInFront;

            }
            if(CosParallaxes.empty())
            {
                continue;
            }

            std::sort(
                    CosParallaxes.begin(),
                    CosParallaxes.end());

            const std::size_t ParallaxIndex =
                std::min<std::size_t>(
                        50,
                        CosParallaxes.size() - 1);

            const fp64 Parallax =
                std::acos(
                        CosParallaxes[ParallaxIndex]) *
                180.0 / M_PI;

            if(Parallax < PANTO_INIT_MIN_PARALLAX_DEGREES)
            {
                continue;
            }

            if(NumPointsInFront >
               BestReconstruction.NumPointsInFront)
            {
                BestReconstruction.R = R;
                BestReconstruction.t = Translation;

                BestReconstruction.MapPoints =
                    std::move(MapPoints);

                BestReconstruction.NumPointsInFront =
                    NumPointsInFront;

                BestReconstruction.Valid = true;
            }
        }

        return BestReconstruction;
    }

private:

    static Model H_EstimateMinimal( const Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS>& Points1, const Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS>& Points2)
    {
        Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS> PointsNormalized1;
        Eigen::Matrix<fp64, 3, PANTO_HOMOGRAPHY_MIN_POINTS> PointsNormalized2;

        Eigen::Matrix3d T1;
        Eigen::Matrix3d T2;

        INITPriv_NormalizePoints2D<PANTO_HOMOGRAPHY_MIN_POINTS>( Points1, PointsNormalized1, T1);

        INITPriv_NormalizePoints2D<PANTO_HOMOGRAPHY_MIN_POINTS>( Points2, PointsNormalized2, T2);

        Eigen::Matrix<fp64, 8, 9> A;

        for(i32 i = 0; i < PANTO_HOMOGRAPHY_MIN_POINTS; ++i)
        {
            const fp64 X1 = PointsNormalized1(0, i);
            const fp64 Y1 = PointsNormalized1(1, i);
            const fp64 X2 = PointsNormalized2(0, i);
            const fp64 Y2 = PointsNormalized2(1, i);

            A.row(2 * i) <<
                -X1, -Y1, -1.0,
                 0.0, 0.0, 0.0,
                 X2 * X1, X2 * Y1, X2;

            A.row(2 * i + 1) <<
                 0.0, 0.0, 0.0,
                -X1, -Y1, -1.0,
                 Y2 * X1, Y2 * Y1, Y2;
        }

        Eigen::JacobiSVD<Eigen::Matrix<fp64, 8, 9>> SVD_A( A, Eigen::ComputeFullV);

        const Eigen::Matrix<fp64, 9, 1> HVector = SVD_A.matrixV().col(8);

        Eigen::Matrix3d HNormalized = Eigen::Map<const Eigen::Matrix<fp64, 3, 3, Eigen::RowMajor>>(HVector.data());

        Model H = T2.inverse() * HNormalized * T1;

        if(std::abs(H(2, 2)) > std::numeric_limits<fp64>::epsilon())
        {
            H /= H(2, 2);
        }
        else
        {
            const fp64 HNorm = H.norm();
            if(HNorm > std::numeric_limits<fp64>::epsilon())
            {
                H /= HNorm;
            }
        }

        return H;
    }

    std::vector<fp64> H_Error(const std::vector<Eigen::Vector2d>& Points1, const std::vector<Eigen::Vector2d>& Points2, const Model& H) 
    {
        assert(Points1.size() == Points2.size());

        std::vector<fp64> Errors(Points1.size());

        const Model HInverse = H.inverse();

        for(std::size_t i = 0; i < Points1.size(); ++i)
        {
            const Eigen::Vector3d Point1( Points1[i].x(), Points1[i].y(), 1.0);
            const Eigen::Vector3d Point2( Points2[i].x(), Points2[i].y(), 1.0);

            const Eigen::Vector3d ProjectedPoint2Homogeneous = H * Point1;
            const Eigen::Vector3d ProjectedPoint1Homogeneous = HInverse * Point2;

            const Eigen::Vector2d ProjectedPoint2( ProjectedPoint2Homogeneous.x() / ProjectedPoint2Homogeneous.z(),
                ProjectedPoint2Homogeneous.y() / ProjectedPoint2Homogeneous.z());

            const Eigen::Vector2d ProjectedPoint1( ProjectedPoint1Homogeneous.x() / ProjectedPoint1Homogeneous.z(),
                ProjectedPoint1Homogeneous.y() / ProjectedPoint1Homogeneous.z());

            const fp64 DistanceSquared12 = (Points2[i] - ProjectedPoint2).squaredNorm();

            const fp64 DistanceSquared21 = (Points1[i] - ProjectedPoint1).squaredNorm();

            Errors[i] = std::max(DistanceSquared12, DistanceSquared21);
        }
        return Errors;
    }
};

void INITPriv_MatchHistoricalFrames(void);
std::vector<u64> INITPriv_STRANSAC(void);
u64 INITPriv_RandomSeed(void);
std::unique_ptr<ImageToImageMapping> INITPriv_ScoredFAndHEstimation(const std::vector<Eigen::Vector2d>& PointFrameNew, const std::vector<Eigen::Vector2d>& PointFrameHistorical);
std::vector<u64> INITPriv_GetCandidateFrameIDs(std::vector<u64> StationaryTrackIDs);
typeInitReconstruction INITPriv_Reconstruct( const typeInitFrame& HistoricalFrame, const typeInitFrame& NewFrame, const std::vector<u64>& StationaryTrackIDs);
typePantoKeypointFrame INITPriv_GetKeyPointFrame(u64 InitFrameID);
void INITPriv_AppendFrame(const std::vector<cv::Point2d>& Points, 
        const cv::Mat& Descriptors, const fp64 TimeStamp, const std::string& ImagePath);

#endif //  INITPRIV_INITIALIZESLAM_HPP_

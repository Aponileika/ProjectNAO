#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <cassert>

using fp64 = double;

std::vector<Eigen::Vector4d> PROJ_TriangulateDLT(const std::vector<Eigen::Vector2d>& Point1,
                                    const std::vector<Eigen::Vector2d>& Point2,
                                    const Eigen::Matrix<fp64, 3, 4> &P1,
                                    const Eigen::Matrix<fp64, 3, 4> &P2) 
{
    assert(Point1.size() == Point2.size());

    Eigen::Matrix4d A;

    const Eigen::RowVector4d P1R0 = P1.row(0);
    const Eigen::RowVector4d P1R1 = P1.row(1);
    const Eigen::RowVector4d P1R2 = P1.row(2);

    const Eigen::RowVector4d P2R0 = P2.row(0);
    const Eigen::RowVector4d P2R1 = P2.row(1);
    const Eigen::RowVector4d P2R2 = P2.row(2);

    std::vector<Eigen::Vector4d> MapPoints(Point1.size());

    Eigen::JacobiSVD<Eigen::Matrix4d, Eigen::ComputeFullV> SVD;
    for(std::size_t i{}; i < Point1.size(); i++)
    {
        A.row(0) = Point1[i].x() * P1R2 - P1R0;
        A.row(1) = Point1[i].y() * P1R2 - P1R1;
        A.row(2) = Point2[i].x() * P2R2 - P2R0;
        A.row(3) = Point2[i].y() * P2R2 - P2R1;

        SVD.compute(A);
        MapPoints[i] = SVD.matrixV().col(3);
    }

    return MapPoints;
}

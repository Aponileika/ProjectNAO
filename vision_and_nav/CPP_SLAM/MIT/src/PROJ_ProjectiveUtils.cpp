#include "PROJ_ProjectiveUtils.hpp"

Eigen::Vector3d PROJ_Homog2Cart(cv::Mat vec) {
  assert(vec.rows == 4 && vec.cols == 1);
  fp64 w = vec.at<fp64>(3, 0);
  return Eigen::Vector3d(vec.at<fp64>(0, 0) / w, vec.at<fp64>(1, 0) / w,
                         vec.at<fp64>(2, 0) / w);
}

Eigen::Vector3d PROJ_Homog2Cart(Eigen::Vector4d vec) {
  fp64 w = vec(3);
  return Eigen::Vector3d(vec(0) / w, vec(1) / w, vec(2) / w);
}

Eigen::Vector2d PROJ_Homog2Cart(Eigen::Vector3d vec) {
  fp64 w = vec(2);
  return Eigen::Vector2d(vec(0) / w, vec(1) / w);
}

Eigen::Vector4d PROJ_CV2NormalizedEigen(cv::Mat vec) {
  assert(vec.rows == 4 && vec.cols == 1);
  Eigen::Vector4d ret(vec.at<fp64>(0, 0), vec.at<fp64>(1, 0),
                      vec.at<fp64>(2, 0), vec.at<fp64>(3, 0));
  fp64 norm = ret.norm();
  return ret / norm;
}

void PROJ_NormalizeToSpherical(Eigen::Vector4d &Vec) { Vec /= Vec.norm(); }

Eigen::Vector4d PROJ_NormalizeToSpherical(const Eigen::Vector4d &Point) {
  return Point / Point.norm();
}

cv::Mat PROJ_ToHomogFromCart(cv::Point2d point) {
  return (cv::Mat_<fp64>(3, 1) << point.x, point.y, 1.0f);
}

Eigen::Matrix3d PROJ_CrossProductMatrix(Eigen::Vector3d vec) {
  Eigen::Matrix3d M;
  M << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;
  return M;
}

/*
 * Get the camera center, assuming T is in CCS
 * */
Eigen::Vector3d PROJ_GetCameraCenter(const Eigen::Matrix4d T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d t = T.block(0, 3, 3, 1);
  return -R.transpose() * t;
}

Eigen::Vector4d PROJ_TriangulateDLT(const Eigen::Vector2d &Point1,
                                    const Eigen::Vector2d &Point2,
                                    const Eigen::Matrix<fp64, 3, 4> &P1,
                                    const Eigen::Matrix<fp64, 3, 4> &P2) {
  Eigen::Matrix4d A;

  const fp64 Point1x = Point1.x();
  const fp64 Point1y = Point1.y();

  const fp64 Point2x = Point2.x();
  const fp64 Point2y = Point2.y();

  const Eigen::RowVector4d P1R0 = P1.row(0);
  const Eigen::RowVector4d P1R1 = P1.row(1);
  const Eigen::RowVector4d P1R2 = P1.row(2);

  const Eigen::RowVector4d P2R0 = P2.row(0);
  const Eigen::RowVector4d P2R1 = P2.row(1);
  const Eigen::RowVector4d P2R2 = P2.row(2);

  A.row(0) = Point1x * P1R2 - P1R0;

  A.row(1) = Point1y * P1R2 - P1R1;

  A.row(2) = Point2x * P2R2 - P2R0;

  A.row(3) = Point2y * P2R2 - P2R1;

  const Eigen::JacobiSVD<Eigen::Matrix4d> SVD(A, Eigen::ComputeFullV);

  Eigen::Vector4d X = SVD.matrixV().col(3);

  X /= X.w();

  return X;
}

std::vector<Eigen::Vector4d> PROJ_TriangulateLOST(
    const std::vector<std::vector<Eigen::Vector3d>> &pixelCoords,
    const std::vector<std::vector<Eigen::Matrix4d>> &T,
    const Eigen::Matrix3d K) {
  /*
   * Assumes all pictures are taken with the same camera (same intrinsics)
   * */
  const u64 numTri = pixelCoords.size();
  std::vector<Eigen::Vector4d> Xh(numTri);
  for (u64 j = 0; j < numTri; j++) {
    const auto &pixels = pixelCoords[j];
    const u64 n = pixels.size();
    const Eigen::Matrix3d Kinv = K.inverse();
    const auto &T_pixels = T[j];
    std::vector<Eigen::Vector3d> aWorld(n);
    std::vector<Eigen::Vector3d> cWorld(n);
    std::vector<Eigen::Vector3d> xBar(n);

    for (u64 i = 0; i < n; i++) {
      Eigen::Vector3d pixel = pixels[i];
      xBar[i] = Kinv * pixel;
      Eigen::Matrix3d Rcw = T_pixels[i].block(0, 0, 3, 3);

      aWorld[i] = (Rcw.transpose() * xBar[i]).normalized();
      cWorld[i] = PROJ_GetCameraCenter(T_pixels[i]);
    }

    Eigen::MatrixXd A(n * 2, 4);

    for (u64 i = 0; i < n; ++i) {
      // Pick another view j with largest ray angle
      int bestK = -1;
      double bestSin = -1.0;

      for (u64 k = 0; k < n; ++k) {
        if (i == k)
          continue;

        fp64 s = aWorld[i].cross(aWorld[k]).norm();

        if (s > bestSin) {
          bestSin = s;
          bestK = k;
        }
      }

      double q = 1.0;

      if (bestK >= 0 && bestSin > 1e-9) {
        Eigen::Vector3d d = cWorld[bestK] - cWorld[i];

        // Law-of-sines range estimate
        double rho_i = d.cross(aWorld[bestK]).norm() / bestSin;

        q = xBar[i].norm() / (PANTO_PIXEL_MEAS_STD_DEV * rho_i);
      }

      Eigen::Matrix<double, 3, 4> P = T_pixels[i].block<3, 4>(0, 0);
      Eigen::Matrix<double, 3, 4> M = PROJ_CrossProductMatrix(xBar[i]) * P;

      // LOST = weighted DLT rows
      A.row(2 * i + 0) = q * M.row(0);
      A.row(2 * i + 1) = q * M.row(1);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

    Xh[j] = svd.matrixV().col(3);
  }
  return Xh;
}

bool PROJ_Project(const Eigen::Vector4d &MapPoint, Eigen::Vector2d &ImagePoint,
                  const typeCamera &Camera) {
  fp64 w = MapPoint.w();
  Eigen::Vector3d NormalizedMapPoint(MapPoint.x() / w, MapPoint.y() / w,
                                     MapPoint.z() / w);

  NormalizedMapPoint = Camera.Pose.R * NormalizedMapPoint + Camera.Pose.t;

  if (NormalizedMapPoint.z() < 0.0f) {
    return false;
  }

  const typeCameraIntrinsics Intrinsics = *Camera.Intrinsics;

  fp64 u = NormalizedMapPoint.x() / NormalizedMapPoint.z();
  fp64 v = NormalizedMapPoint.y() / NormalizedMapPoint.z();

  const Eigen::Vector3d Pixel = Intrinsics.K * Eigen::Vector3d{u, v, 1.0};

  ImagePoint = Pixel.head<2>();

  if (ImagePoint.x() < 0.0 || ImagePoint.x() >= PANTO_IMAGE_WIDTH ||
      ImagePoint.y() < 0.0 || ImagePoint.y() >= PANTO_IMAGE_HEIGHT) {
    return false;
  }

  return true;
}

#ifndef __OP_BA_HPP_
#define __OP_BA_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "Config.hpp"
#include "MAP_Mapping.hpp"
#include "PANTOVEC_PantoVector.hpp"

typedef enum 
{
    OptimizationTypePoseAndPoints = 0,
    OptimizationTypePose = 1,
    OptimizationTypeTracking = 2,
    OptimizationTypeLocal = 3
}typeOptimizationTarget;

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

struct typeOPCameraIntrinsics
{
    typeOPCameraIntrinsics(Eigen::Matrix3d K) :
        fx(K(0, 0)), fy(K(1, 1)), cx(K(0, 2)), cy(K(1, 2)) {}
     fp64 fx, fy, cx, cy;
};

struct OP_ReprojectionError
{
    OP_ReprojectionError(fp64 ObservedX, fp64 ObservedY, 
            const struct typeOPCameraIntrinsics* Intrinsics)
        : ObservedX_(ObservedX), ObservedY_(ObservedY),
            Intrinsics_(Intrinsics){}

    template <typename T>
    bool operator()(const T* const qp,
                    const T* const tp,
                    const T* const Xp, 
                    T* residuals) const{
        Eigen::Map<const Eigen::Quaternion<T>> qwc(qp);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(tp);
        Eigen::Map<const Eigen::Matrix<T,4,1>> X(Xp);

        //Note we do not use homogenous coordinates to project, 
        //Right now we distort all projected points, a more reasonable
        //approach could be to distort all detected ORB keypoints instead
        //Since this would save some computations

        //IREG P.176, since rotation is parametrized as 
        //R^T, and translation as -R^T*t.
        const Eigen::Quaternion<T> qcw = qwc.conjugate();

        const fp64 fx_ = Intrinsics_->fx;
        const fp64 fy_ = Intrinsics_->fy;
        const fp64 cx_ = Intrinsics_->cx;
        const fp64 cy_ = Intrinsics_->cy;

        //X in camera coordinates is now R^T(X - t)
        Eigen::Matrix<T, 3, 1> XYZ(
                X(0),
                X(1),
                X(2));
        T W = X(3);
        Eigen::Matrix<T, 3, 1> Xc = qcw * (XYZ - t * W);

        //projection
        T x = Xc.x() / Xc.z();
        T y = Xc.y() / Xc.z();

        //Normalized image coordinates, u,v convention from graphics programming?
        T u = T(fx_) * x + T(cx_);
        T v = T(fy_) * y + T(cy_);

        residuals[0] = u - T(ObservedX_);
        residuals[1] = v - T(ObservedY_);
        return true;
    }
    static ceres::CostFunction* Create(const fp64 ObservedX, const fp64 ObservedY, const struct typeOPCameraIntrinsics* Intrinsics)
    {
        return new ceres::AutoDiffCostFunction<OP_ReprojectionError, 2, 4, 3, 4>(
            new OP_ReprojectionError(ObservedX, ObservedY, Intrinsics));
    }

    fp64 ObservedX_, ObservedY_;
    const struct typeOPCameraIntrinsics* Intrinsics_;
};

void OP_BundleAdjust(typeGlobalMap& Map, typeOptimizationTarget Target, const typeLocalMap& LocalMap, typeKeyFrame* NewKeyFrame);

#endif //__OP_BA_HPP_

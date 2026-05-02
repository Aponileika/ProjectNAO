#ifndef __OP_BA_HPP_
#define __OP_BA_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include "CArenaAlloc.h"
#include "CM_Camera.hpp"
#include "PT_Points.hpp"
#include "VW_Views.hpp"
#include "OB_Observations.hpp"

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

struct OPIntrinsics
{
    OPIntrinsics(struct CameraIntrinsics* ci) :
        fx(ci->K(0, 0)), fy(ci->K(1, 1)), cx(ci->K(0, 2)), cy(ci->K(1, 2)) {}
     fp64 fx, fy, cx, cy;
};

struct ReprojectionError
{
    ReprojectionError(fp64 observedx, fp64 observedy, 
            const struct OPIntrinsics* Intr)
        : observedx_(observedx), observedy_(observedy),
            intr_(Intr){}

    template <typename T>
    bool operator()(const T* const qp,
                    const T* const tp,
                    const T* const Xp, 
                    T* residuals) const{
        Eigen::Map<const Eigen::Quaternion<T>> qwc(qp);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(tp);
        Eigen::Map<const Eigen::Matrix<T,3,1>> X(Xp);

        //Note we do not use homogenous coordinates to project, 
        //Right now we distort all projected points, a more reasonable
        //approach could be to distort all detected ORB keypoints instead
        //Since this would save some computations

        //IREG P.176, since rotation is parametrized as 
        //R^T, and translation as -R^T*t.
        const Eigen::Quaternion<T> qcw = qwc.conjugate();

        const fp64 fx_ = intr_->fx;
        const fp64 fy_ = intr_->fy;
        const fp64 cx_ = intr_->cx;
        const fp64 cy_ = intr_->cy;

        //X in camera coordinates is now R^T(X - t)
        Eigen::Matrix<T, 3, 1> Xc = qcw * (X - t);

        //projection
        T x = Xc.x() / Xc.z();
        T y = Xc.y() / Xc.z();

        //Normalized image coordinates, u,v convention from graphics programming?
        T u = T(fx_) * x + T(cx_);
        T v = T(fy_) * y + T(cy_);

        residuals[0] = u - T(observedx_);
        residuals[1] = v - T(observedy_);
        return true;
    }
    static ceres::CostFunction* Create(const fp64 observed_x, const fp64 observed_y, const struct OPIntrinsics* intr)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
            new ReprojectionError(observed_x, observed_y, intr));
    }

    fp64 observedx_, observedy_;
    const struct OPIntrinsics* intr_;
};

void OP_BundleAdjust(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points);

#endif //__OP_BA_HPP_

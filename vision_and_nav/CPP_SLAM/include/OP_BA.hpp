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
        fx(ci->K(0, 0)), fy(ci->K(1, 1)), cx(ci->K(0, 2)), cy(ci->K(1, 2)), 
        k1(ci->distcoeffs[0]),
        k2(ci->distcoeffs[1]), 
        p1(ci->distcoeffs[2]), 
        p2(ci->distcoeffs[3]),
        k3(ci->distcoeffs[4]) {}
     fp64 fx, fy, cx, cy;
     fp64 k1, k2, p1, p2, k3;
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

        const fp64 k1_ = intr_->k1;
        const fp64 k2_ = intr_->k2;
        const fp64 p1_ = intr_->p1;
        const fp64 p2_ = intr_->p2;
        const fp64 k3_ = intr_->k3;


        //X in camera coordinates is now R^T(X - t)
        Eigen::Matrix<T, 3, 1> Xc = qcw * (X - t);

        //projection
        T x = Xc.x() / Xc.z();
        T y = Xc.y() / Xc.z();

        // https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
        T r2 = x*x + y*y;

        T radial = T(1.0) + T(k1_)*r2 + T(k2_)*r2*r2 + T(k3_)*r2*r2*r2;

        T x_tan = T(2.0)*T(p1_)*x*y + T(p2_)*(r2 + T(2.0)*x*x);
        T y_tan = T(p1_)*(r2 + T(2.0)*y*y) + T(2.0)*T(p2_)*x*y;

        //Distort the projected image coordinates, to match the observedx and y
        T xd = x * radial + x_tan;
        T yd = y * radial + y_tan;

        //Normalized image coordinates, u,v convention from graphics programming?
        T u = T(fx_) * xd + T(cx_);
        T v = T(fy_) * yd + T(cy_);

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

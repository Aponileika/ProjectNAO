#include "../include/OP_BA.hpp"

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

//read only intrinsics for residual calculation
static const struct OPIntrinsics opintr(CM_GetIntrinsics());

void __OP_BuildProblem(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points,
        ceres::Problem* problem);

void OP_BundleAdjust(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points)
{
    ceres::Problem problem;
    __OP_BuildProblem(views, obs, points, &problem);
    ceres::Solver::Options options;
    options.max_num_iterations = CERES_MAX_ITER;
    options.minimizer_progress_to_stdout = true;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = CERES_NUM_THREADS;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    for(size_t i = 0; i < views->views.size(); i++)
    {
        CM_SetRtfromParam(&views->views[i]);
    }
    LG_Log(LogSeverity::DBG, "%s\n", summary.FullReport().c_str());
}

void __OP_BuildProblem(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points,
        ceres::Problem* problem)
{
    /*See https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#manifold
     *Optimizing on manifolds seemingly has many benefits, one of them is that
     *Quaternions are naturally constrained to be normalized, another is that
     *The dimension of the optimization is reduced to its natural size,
     *I am unsure but I think this means that if we have 3DOF but 4 parameters
     *the optimization is reduced to the natural size of optimizing with 3DOF, whatever that means?
    */
    //We need to add the camera parameters first so we can
    //Set the first camera constant
    for (size_t i = 0; i < views->views.size(); ++i) 
    {
        Param* p = views->views[i].p;

        problem->AddParameterBlock(p->q.coeffs().data(), 4);
        problem->SetManifold(p->q.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        problem->AddParameterBlock(p->t.data(), 3);

        if (i == 0) 
        {
            problem->SetParameterBlockConstant(p->q.coeffs().data());
            problem->SetParameterBlockConstant(p->t.data());
        }
    }

    for(size_t i = 0; i < points->points.size(); i++)
    {
        Eigen::Vector4d& vec = points->points[i];

        problem->AddParameterBlock(vec.data(), 4);
        problem->SetManifold(vec.data(), new ceres::SphereManifold<4>());
    }

    for (size_t i = 0; i < obs->observations.size(); ++i) 
    {
        const fp64 px = obs->observations[i].x;
        const fp64 py = obs->observations[i].y;

        ceres::CostFunction* costfunc =
            ReprojectionError::Create(px, py, &opintr);

        ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

        u64 viewidx = obs->view_indexes[i];
        u64 pidx = obs->point_indexes[i];
        Param* p = views->views[viewidx].p;

        problem->AddResidualBlock(costfunc,
                                  lossfunc,
                                  p->q.coeffs().data(),
                                  p->t.data(),
                                  points->points[pidx].data());
    }
}

void __OP_BuildProblemLastCam(struct ViewSet* views, struct ObservationSet* obs, struct PointSet* points,
        ceres::Problem* problem)
{
    /*See https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#manifold
     *Optimizing on manifolds seemingly has many benefits, one of them is that
     *Quaternions are naturally constrained to be normalized, another is that
     *The dimension of the optimization is reduced to its natural size,
     *I am unsure but I think this means that if we have 3DOF but 4 parameters
     *the optimization is reduced to the natural size of optimizing with 3DOF, whatever that means?
    */
    //We need to add the camera parameters first so we can
    //Set the first camera constant
    for (size_t i = 0; i < views->views.size(); ++i) 
    {
        Param* p = views->views[i].p;

        problem->AddParameterBlock(p->q.coeffs().data(), 4);
        problem->SetManifold(p->q.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        problem->AddParameterBlock(p->t.data(), 3);

        if (i == 0) 
        {
            problem->SetParameterBlockConstant(p->q.coeffs().data());
            problem->SetParameterBlockConstant(p->t.data());
        }
    }

    for(size_t i = 0; i < points->points.size(); i++)
    {
        Eigen::Vector4d& vec = points->points[i];

        problem->AddParameterBlock(vec.data(), 4);
        problem->SetManifold(vec.data(), new ceres::SphereManifold<4>());
    }

    for (size_t i = 0; i < obs->observations.size(); ++i) 
    {
        const fp64 px = obs->observations[i].x;
        const fp64 py = obs->observations[i].y;

        ceres::CostFunction* costfunc =
            ReprojectionError::Create(px, py, &opintr);

        ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

        u64 viewidx = obs->view_indexes[i];
        u64 pidx = obs->point_indexes[i];
        Param* p = views->views[viewidx].p;

        problem->AddResidualBlock(costfunc,
                                  lossfunc,
                                  p->q.coeffs().data(),
                                  p->t.data(),
                                  points->points[pidx].data());
    }
}

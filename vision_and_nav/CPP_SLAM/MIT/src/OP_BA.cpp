#include "../include/OP_BA.hpp"

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

//read only intrinsics for residual calculation
static const struct typeOPCameraIntrinsics OPCameraIntrinsics(CM_GetIntrinsics()->K);

void __OP_BuildProblem(typeGlobalMap& Map, ceres::Problem& Problem);
void __OP_BuildProblemPoseOnly(typeGlobalMap& Map, ceres::Problem& Problem);
void __OP_BuildProblemTracking(typeGlobalMap& Map, ceres::Problem& Problem);
void __OP_BuildProblemLocal(typeGlobalMap& Map, ceres::Problem& Problem, typeLocalMap& LocalMap);

void OP_BundleAdjust(typeGlobalMap& Map, typeOptimizationTarget Target, typeLocalMap LocalMap)
{
    ceres::Problem Problem;
    ceres::Solver::Options options;
    options.max_num_iterations = CERES_MAX_ITER;
    options.minimizer_progress_to_stdout = true;

    options.num_threads = CERES_NUM_THREADS;

    switch(Target)
    {
        case typePoseAndPoints:
            __OP_BuildProblem(Map, Problem);
            options.linear_solver_type = ceres::SPARSE_SCHUR;
        case typePose:
            __OP_BuildProblemPoseOnly(Map, Problem);
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        case typeTracking:
            __OP_BuildProblemTracking(Map, Problem);
            options.linear_solver_type = ceres::DENSE_QR;
        case typeLocal:
            __OP_BuildProblemLocal(Map, Problem, LocalMap);
            options.linear_solver_type = ceres::DENSE_SCHUR;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &Problem, &summary);
    for(typeKeyFrame& KeyFrame : Map.KeyFrames)
    {
        CM_SetRtfromParam(&KeyFrame.Pose);
    }
    LG_Log(LogSeverity::DBG, "%s\n", summary.FullReport().c_str());
}

void __OP_BuildProblem(typeGlobalMap& Map, ceres::Problem& Problem)
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
    for (typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

        Problem.AddParameterBlock(Parameters.q.coeffs().data(), 4);
        Problem.SetManifold(Parameters.q.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        Problem.AddParameterBlock(Parameters.t.data(), 3);

        if (KeyFrame.ID == 0) 
        {
            Problem.SetParameterBlockConstant(Parameters.q.coeffs().data());
            Problem.SetParameterBlockConstant(Parameters.t.data());
        }
    }

    for(typePantoMapPoint& MapPoint : Map.MapPoints)
    {
        Problem.AddParameterBlock(MapPoint.Point.data(), 4);
        Problem.SetManifold(MapPoint.Point.data(), new ceres::SphereManifold<4>());
    }

    for(typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        for(std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
        {
            for(typePantoImagePoint& ImagePoint : CellPoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET)
                {
                    const fp64 PointX = ImagePoint.Point.x();
                    const fp64 PointY = ImagePoint.Point.y();

                    ceres::CostFunction* costfunc =
                        OP_ReprojectionError::Create(PointX, PointY, &OPCameraIntrinsics);

                    ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

                    typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

                    Problem.AddResidualBlock(costfunc,
                                              lossfunc,
                                              Parameters.q.coeffs().data(),
                                              Parameters.t.data(),
                                              Map.MapPoints[MapPointID].Point.data());
                }
            }
        }
    }
}

void __OP_BuildProblemPoseOnly(typeGlobalMap& Map, ceres::Problem& Problem)
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
    for (typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

        Problem.AddParameterBlock(Parameters.q.coeffs().data(), 4);
        Problem.SetManifold(Parameters.q.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        Problem.AddParameterBlock(Parameters.t.data(), 3);

        if (KeyFrame.ID == 0) 
        {
            Problem.SetParameterBlockConstant(Parameters.q.coeffs().data());
            Problem.SetParameterBlockConstant(Parameters.t.data());
        }
    }

    for(typePantoMapPoint& MapPoint : Map.MapPoints)
    {
        Problem.AddParameterBlock(MapPoint.Point.data(), 4);
        Problem.SetParameterBlockConstant(MapPoint.Point.data());
    }

    for(typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        for(std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
        {
            for(typePantoImagePoint& ImagePoint : CellPoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET)
                {
                    const fp64 PointX = ImagePoint.Point.x();
                    const fp64 PointY = ImagePoint.Point.y();

                    ceres::CostFunction* costfunc =
                        OP_ReprojectionError::Create(PointX, PointY, &OPCameraIntrinsics);

                    ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

                    typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

                    Problem.AddResidualBlock(costfunc,
                                              lossfunc,
                                              Parameters.q.coeffs().data(),
                                              Parameters.t.data(),
                                              Map.MapPoints[MapPointID].Point.data());
                }
            }
        }
    }
}

void __OP_BuildProblemTracking(typeGlobalMap& Map, ceres::Problem& Problem)
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

    typeKeyFrame& KeyFrame = Map.KeyFrames.back();
    assert(KeyFrame.ID == PANTO_ID_NOT_SET);
    typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

    Problem.AddParameterBlock(Parameters.q.coeffs().data(), 4);
    Problem.SetManifold(Parameters.q.coeffs().data(),
                         new ceres::EigenQuaternionManifold());

    Problem.AddParameterBlock(Parameters.t.data(), 3);

    for (std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
    {
        for (typePantoImagePoint& ImagePoint : CellPoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID != PANTO_ID_NOT_SET)
            {
                typePantoMapPoint& MapPoint =
                    Map.MapPoints[MapPointID];

                Problem.AddParameterBlock(
                        MapPoint.Point.data(), 4);

                Problem.SetParameterBlockConstant(
                        MapPoint.Point.data());

                ceres::CostFunction* CostFunc =
                    OP_ReprojectionError::Create(
                            ImagePoint.Point.x(),
                            ImagePoint.Point.y(),
                            &OPCameraIntrinsics);

                ceres::LossFunction* LossFunc =
                    new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

                Problem.AddResidualBlock(
                        CostFunc,
                        LossFunc,
                        Parameters.q.coeffs().data(),
                        Parameters.t.data(),
                        MapPoint.Point.data());
            }
        }
    }
}

void __OP_BuildProblemLocal(typeGlobalMap& Map, ceres::Problem& Problem, typeLocalMap& LocalMap)
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
    for(typeKeyFrame& KeyFrame : LocalMap.KeyFrames) 
    {
        typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

        Problem.AddParameterBlock(Parameters.q.coeffs().data(), 4);
        Problem.SetManifold(Parameters.q.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        Problem.AddParameterBlock(Parameters.t.data(), 3);

        if (KeyFrame.ID == 0) 
        {
            Problem.SetParameterBlockConstant(Parameters.q.coeffs().data());
            Problem.SetParameterBlockConstant(Parameters.t.data());
        }
    }

    for(const typeKeyFrame& KeyFrame: LocalMap.KeyFrames)
    {
        for(const std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
        {
            for(const typePantoImagePoint& ImagePoint : CellPoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET)
                {
                    Problem.AddParameterBlock(Map.MapPoints[MapPointID].Point.data(), 4);
                    Problem.SetManifold(Map.MapPoints[MapPointID].Point.data(), new ceres::SphereManifold<4>());
                }
                else
                {
                    // Dont optimize points with no associated 2d point
                    Problem.AddParameterBlock(Map.MapPoints[MapPointID].Point.data(), 4);
                    Problem.SetParameterBlockConstant(Map.MapPoints[MapPointID].Point.data());
                }
            }
        }
    }

    for(typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        for(std::vector<typePantoImagePoint>& CellPoints : KeyFrame.Points)
        {
            for(typePantoImagePoint& ImagePoint : CellPoints)
            {
                const u64 MapPointID = ImagePoint.MapPointID;
                if(MapPointID != PANTO_ID_NOT_SET)
                {
                    const fp64 PointX = ImagePoint.Point.x();
                    const fp64 PointY = ImagePoint.Point.y();

                    ceres::CostFunction* costfunc =
                        OP_ReprojectionError::Create(PointX, PointY, &OPCameraIntrinsics);

                    ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

                    typePoseParameters& Parameters = KeyFrame.Pose.Parameters;

                    Problem.AddResidualBlock(costfunc,
                                              lossfunc,
                                              Parameters.q.coeffs().data(),
                                              Parameters.t.data(),
                                              Map.MapPoints[MapPointID].Point.data());
                }
            }
        }
    }
}

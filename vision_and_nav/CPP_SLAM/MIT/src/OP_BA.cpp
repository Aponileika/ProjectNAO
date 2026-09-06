#include "../include/OP_BA.hpp"
#include "IMU_PreIntegration.hpp"
#include <cmath>

/*see https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *and https://ceres-solver.readthedocs.io/latest/nnls_tutorial.html
 *for details into ceres nonlinear solving for BA
 *For the current simple implementation https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/simple_bundle_adjuster.cc
 *Is also nice to follow
 * */

//read only intrinsics for residual calculation
static const struct typeOPCameraIntrinsics OPCameraIntrinsics(CM_GetIntrinsics()->K);
static const Eigen::Matrix4d* TBS = &CM_GetIntrinsics()->T_BS;

void __OP_BuildProblem(typeGlobalMap& Map, ceres::Problem& Problem);
void __OP_BuildProblemPoseOnly(typeGlobalMap& Map, ceres::Problem& Problem);
void __OP_BuildProblemTracking(typeGlobalMap& Map, ceres::Problem& Problem,
        typeKeyFrame* NewKeyFrame, typeKeyFrame* PreviousFrame);
void __OP_BuildProblemLocal(typeGlobalMap& Map, ceres::Problem& Problem, const typeLocalMap& LocalMap);

void OP_BundleAdjust(typeGlobalMap& Map, typeOptimizationTarget Target,
        const typeLocalMap& LocalMap, typeKeyFrame* NewKeyFrame,
        typeKeyFrame* PreviousFrame)
{
    ceres::Problem Problem;
    ceres::Solver::Options options;
    options.max_num_iterations = CERES_MAX_ITER;
    options.minimizer_progress_to_stdout = false;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    options.num_threads = CERES_NUM_THREADS;

    switch(Target)
    {
        case OptimizationTypePoseAndPoints:
            __OP_BuildProblem(Map, Problem);
            options.linear_solver_type = ceres::SPARSE_SCHUR;
            break;
        case OptimizationTypePose:
            __OP_BuildProblemPoseOnly(Map, Problem);
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            break;
        case OptimizationTypeTracking:
            __OP_BuildProblemTracking(Map, Problem, NewKeyFrame, PreviousFrame);
            options.linear_solver_type = ceres::DENSE_QR;
            break;
        case OptimizationTypeLocal:
            __OP_BuildProblemLocal(Map, Problem, LocalMap);
            options.linear_solver_type = ceres::DENSE_SCHUR;
            break;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &Problem, &summary);
    if(Target == OptimizationTypeTracking)
    {
        CM_SetRtfromParam(&(NewKeyFrame->Camera));
#if defined(CONFIG_IMU)
        KEY_UpdateNavState(NewKeyFrame);
#endif
    }
    else if(Target == OptimizationTypeLocal)
    {
        for(const u64 KeyFrameID : LocalMap.KeyFrameIDs)
        {
            CM_SetRtfromParam(&Map.KeyFrames[KeyFrameID].Camera);
#if defined(CONFIG_IMU)
            KEY_UpdateNavState(&Map.KeyFrames[KeyFrameID]);
#endif
        }
    }
    else
    {
        for(typeKeyFrame& KeyFrame : Map.KeyFrames)
        {
            CM_SetRtfromParam(&KeyFrame.Camera);
#if defined(CONFIG_IMU)
            KEY_UpdateNavState(&KeyFrame);
#endif
        }
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
#if defined(CONFIG_IMU)
    const Eigen::Vector3d Grav = *IMU_GetGravity();
#endif

    // Add every camera, and every navigation state for VI optimization,
    // before creating residuals between keyframes.
    for (typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        typeCameraPose& Parameters = KeyFrame.Camera.Pose;

        Problem.AddParameterBlock(Parameters.Quaternion.coeffs().data(), 4);
        Problem.SetManifold(Parameters.Quaternion.coeffs().data(),
                             new ceres::EigenQuaternionManifold());

        Problem.AddParameterBlock(Parameters.tParametrization.data(), 3);

#if defined(CONFIG_IMU)
        typeNavigationState& NavigationState = KeyFrame.NavigationState;
        Problem.AddParameterBlock(NavigationState.Velocity.data(), 3);
        Problem.AddParameterBlock(NavigationState.GyroBias.data(), 3);
        Problem.AddParameterBlock(
                NavigationState.AccelorometerBias.data(), 3);
#endif

        if (KeyFrame.ID == 0) 
        {
            Problem.SetParameterBlockConstant(Parameters.Quaternion.coeffs().data());
            Problem.SetParameterBlockConstant(Parameters.tParametrization.data());
#if defined(CONFIG_IMU)
            Problem.SetParameterBlockConstant(NavigationState.Velocity.data());
            Problem.SetParameterBlockConstant(NavigationState.GyroBias.data());
            Problem.SetParameterBlockConstant(
                    NavigationState.AccelorometerBias.data());
#endif
        }
    }

    for(typePantoMapPoint& MapPoint : Map.MapPoints)
    {
        if(PT_GetNumObservations(MapPoint) > 1)
        {
            Problem.AddParameterBlock(MapPoint.Point.data(), 4);
            Problem.SetManifold(MapPoint.Point.data(), new ceres::SphereManifold<4>());
        }
    }

    for(typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        typeCameraPose& Parameters = KeyFrame.Camera.Pose;

        for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;
            if(MapPointID != PANTO_ID_NOT_SET)
            {
                if(PT_GetNumObservations(Map.MapPoints[MapPointID]) <= 1)
                {
                    continue;
                }

                const fp64 PointX = ImagePoint.Point.x();
                const fp64 PointY = ImagePoint.Point.y();

                ceres::CostFunction* costfunc =
                    OP_ReprojectionError::Create(PointX, PointY, &OPCameraIntrinsics);

                ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

                Problem.AddResidualBlock(costfunc,
                                          lossfunc,
                                          Parameters.Quaternion.coeffs().data(),
                                          Parameters.tParametrization.data(),
                                          Map.MapPoints[MapPointID].Point.data());
            }
        }

#if defined(CONFIG_IMU)
        if(KeyFrame.ID == 0)
        {
            continue;
        }

        assert(KeyFrame.PreviousKFID != PANTO_ID_NOT_SET);
        assert(Map.KeyFrames.contains(KeyFrame.PreviousKFID));

        typeKeyFrame& PreviousKeyFrame =
            Map.KeyFrames[KeyFrame.PreviousKFID];
        typeCameraPose& PreviousCameraParameters =
            PreviousKeyFrame.Camera.Pose;
        typeNavigationState& PreviousNavigationState =
            PreviousKeyFrame.NavigationState;
        typeNavigationState& NavigationState = KeyFrame.NavigationState;
        typePreIntegrationData& PreIntegrationData =
            KeyFrame.PreIntegrationData;

        const Eigen::LLT<Eigen::Matrix<fp64, 15, 15>> LLT(
                PreIntegrationData.Covariance);
        const Eigen::Matrix<fp64, 15, 15> L = LLT.matrixL();
        const Eigen::Matrix<fp64, 15, 15> SqrtInfo =
            L.triangularView<Eigen::Lower>().solve(
                    Eigen::Matrix<fp64, 15, 15>::Identity());

        ceres::CostFunction* IMUCostFunction = OP_IMUResidual::Create(
                PreIntegrationData,
                SqrtInfo,
                Grav,
                TBS);

        Problem.AddResidualBlock(
                IMUCostFunction,
                nullptr,
                PreviousCameraParameters.Quaternion.coeffs().data(),
                PreviousCameraParameters.tParametrization.data(),
                PreviousNavigationState.Velocity.data(),
                PreviousNavigationState.GyroBias.data(),
                PreviousNavigationState.AccelorometerBias.data(),
                Parameters.Quaternion.coeffs().data(),
                Parameters.tParametrization.data(),
                NavigationState.Velocity.data(),
                NavigationState.GyroBias.data(),
                NavigationState.AccelorometerBias.data());
#endif
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

#if defined(CONFIG_IMU)
    const Eigen::Vector3d Grav = *IMU_GetGravity();
#endif

    for (typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        const u64 KeyFrameID = KeyFrame.ID;

        assert(Map.KeyFrames.contains(KeyFrameID));

        typeCameraPose& CameraParameters = Map.KeyFrames[KeyFrameID].Camera.Pose;
#if defined(CONFIG_IMU)
        typeNavigationState& IMUParameters = Map.KeyFrames[KeyFrameID].NavigationState;
#endif

        Problem.AddParameterBlock(CameraParameters.Quaternion.coeffs().data(), 4);
        Problem.SetManifold(CameraParameters.Quaternion.coeffs().data(), new ceres::EigenQuaternionManifold());
        Problem.AddParameterBlock(CameraParameters.tParametrization.data(), 3);

#if defined(CONFIG_IMU)
        Problem.AddParameterBlock(IMUParameters.Velocity.data(), 3);
        Problem.AddParameterBlock(IMUParameters.GyroBias.data(), 3);
        Problem.AddParameterBlock(IMUParameters.AccelorometerBias.data(), 3);
#endif

        if(KeyFrameID == 0) 
        {
            Problem.SetParameterBlockConstant(CameraParameters.Quaternion.coeffs().data());
            Problem.SetParameterBlockConstant(CameraParameters.tParametrization.data());
#if defined(CONFIG_IMU)
            Problem.SetParameterBlockConstant(IMUParameters.Velocity.data());
            Problem.SetParameterBlockConstant(IMUParameters.GyroBias.data());
            Problem.SetParameterBlockConstant(IMUParameters.AccelorometerBias.data());
#endif
            continue;
        }

#if defined(CONFIG_IMU)
        typeKeyFrame& PreviousKeyFrame = Map.KeyFrames[Map.KeyFrames[KeyFrameID].PreviousKFID];

        typeCameraPose& PreviousCameraParameters =   PreviousKeyFrame.Camera.Pose;
        typeNavigationState& PreviousIMUParameters = PreviousKeyFrame.NavigationState;

        Problem.AddParameterBlock(PreviousCameraParameters.Quaternion.coeffs().data(), 4);
        Problem.SetManifold(PreviousCameraParameters.Quaternion.coeffs().data(), new ceres::EigenQuaternionManifold());
        Problem.AddParameterBlock(PreviousCameraParameters.tParametrization.data(), 3);

        Problem.AddParameterBlock(PreviousIMUParameters.Velocity.data(), 3);
        Problem.AddParameterBlock(PreviousIMUParameters.GyroBias.data(), 3);
        Problem.AddParameterBlock(PreviousIMUParameters.AccelorometerBias.data(), 3);
#endif
    }

    for(typePantoMapPoint& MapPoint : Map.MapPoints)
    {
        if(PT_GetNumObservations(MapPoint) > 1)
        {
            Problem.AddParameterBlock(MapPoint.Point.data(), 4);
            Problem.SetParameterBlockConstant(MapPoint.Point.data());
        }
    }

    // Add the parameters for IMU, velocity, and biases, pose is shared.
    // Calculate sqrtinfo S where S^T * S = P^-1, using cholesky decomp:
    // P = LL^T, S = L^-1, since S^T * S = L^(-T) * L ^(-1) = P^-1,
    
    ceres::LossFunction* lossfunc = new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

    for(typeKeyFrame& KeyFrame : Map.KeyFrames) 
    {
        typeCameraPose& CameraParameters = KeyFrame.Camera.Pose;

        for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;
            if(MapPointID != PANTO_ID_NOT_SET)
            {
                if(PT_GetNumObservations(Map.MapPoints[MapPointID]) <= 1)
                {
                    continue;
                }

                const fp64 PointX = ImagePoint.Point.x();
                const fp64 PointY = ImagePoint.Point.y();

                ceres::CostFunction* costfunc =
                    OP_ReprojectionError::Create(PointX, PointY, &OPCameraIntrinsics);

                Problem.AddResidualBlock(costfunc,
                                          lossfunc,
                                          CameraParameters.Quaternion.coeffs().data(),
                                          CameraParameters.tParametrization.data(),
                                          Map.MapPoints[MapPointID].Point.data());
            }
        }
        if(KeyFrame.ID == 0)
        {
            continue;
        }

#if defined(CONFIG_IMU)
        typeNavigationState& IMUParameters = KeyFrame.NavigationState;
        typeCameraPose& PreviousCameraParameters = Map.KeyFrames[KeyFrame.PreviousKFID].Camera.Pose;
        typeNavigationState& PreviousIMUParameters = Map.KeyFrames[KeyFrame.PreviousKFID].NavigationState;
        typePreIntegrationData& PreviousData = KeyFrame.PreIntegrationData;

        const Eigen::LLT<Eigen::Matrix<fp64, 15, 15>> LLT(PreviousData.Covariance);
        const Eigen::Matrix<fp64, 15, 15> L = LLT.matrixL();
        const Eigen::Matrix<fp64, 15, 15> SqrtInfo = L.triangularView<Eigen::Lower>().solve(
                Eigen::Matrix<fp64, 15, 15>::Identity());
        ceres::CostFunction* CostFuncIMU = OP_IMUResidual::Create(PreviousData, SqrtInfo, Grav, TBS);


        Problem.AddResidualBlock(CostFuncIMU,
                                    nullptr,
                                    PreviousCameraParameters.Quaternion.coeffs().data(),
                                    PreviousCameraParameters.tParametrization.data(),
                                    PreviousIMUParameters.Velocity.data(),
                                    PreviousIMUParameters.GyroBias.data(),
                                    PreviousIMUParameters.AccelorometerBias.data(),

                                    CameraParameters.Quaternion.coeffs().data(),
                                    CameraParameters.tParametrization.data(),
                                    IMUParameters.Velocity.data(),
                                    IMUParameters.GyroBias.data(),
                                    IMUParameters.AccelorometerBias.data()
                                    );
#endif
    }
}

void __OP_BuildProblemTracking(typeGlobalMap& Map, ceres::Problem& Problem,
        typeKeyFrame* NewKeyFrame, typeKeyFrame* PreviousFrame)
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

    typeKeyFrame* KeyFrame = NewKeyFrame;
    typeCameraPose& CameraParameters = KeyFrame->Camera.Pose;
#if defined(CONFIG_IMU)
    const Eigen::Vector3d Grav = *IMU_GetGravity();
    typeNavigationState& IMUParameters = KeyFrame->NavigationState;

    assert(PreviousFrame != nullptr);
    typeKeyFrame& PreviousKeyFrame = *PreviousFrame;
    typeCameraPose& PreviousCameraParameters = PreviousKeyFrame.Camera.Pose;
    typeNavigationState& PreviousIMUParameters = PreviousKeyFrame.NavigationState;
    typePreIntegrationData& PreviousData = KeyFrame->PreIntegrationData;
#else
    (void)PreviousFrame;
#endif

    Problem.AddParameterBlock(CameraParameters.Quaternion.coeffs().data(), 4);
    Problem.SetManifold(CameraParameters.Quaternion.coeffs().data(),
                         new ceres::EigenQuaternionManifold());

    Problem.AddParameterBlock(CameraParameters.tParametrization.data(), 3);

#if defined(CONFIG_IMU)
    Problem.AddParameterBlock(IMUParameters.Velocity.data(), 3);
    Problem.AddParameterBlock(IMUParameters.GyroBias.data(), 3);
    Problem.AddParameterBlock(IMUParameters.AccelorometerBias.data(), 3);

    Problem.AddParameterBlock(PreviousCameraParameters.Quaternion.coeffs().data(), 4);
    Problem.SetManifold(PreviousCameraParameters.Quaternion.coeffs().data(),
                         new ceres::EigenQuaternionManifold());

    Problem.AddParameterBlock(PreviousCameraParameters.tParametrization.data(), 3);

    Problem.AddParameterBlock(PreviousIMUParameters.Velocity.data(), 3);
    Problem.AddParameterBlock(PreviousIMUParameters.GyroBias.data(), 3);
    Problem.AddParameterBlock(PreviousIMUParameters.AccelorometerBias.data(), 3);

    Problem.SetParameterBlockConstant(PreviousCameraParameters.Quaternion.coeffs().data());
    Problem.SetParameterBlockConstant(PreviousCameraParameters.tParametrization.data());
    Problem.SetParameterBlockConstant(PreviousIMUParameters.Velocity.data());
    Problem.SetParameterBlockConstant(PreviousIMUParameters.GyroBias.data());
    Problem.SetParameterBlockConstant(PreviousIMUParameters.AccelorometerBias.data());
#endif

    u64 NumAssociatedMapPoints = 0;

    for(const typePantoImagePoint& ImagePoint : KeyFrame->Points.ImagePoints)
    {
        if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
        {
            NumAssociatedMapPoints++;
        }
    }

    LG_Log( LogSeverity::DBG, "[__OP_BuildProblemTracking] KeyFrame has %llu associated map points\n", static_cast<unsigned long long>(NumAssociatedMapPoints));

    ceres::LossFunction* LossFunc =
        new ceres::HuberLoss(CERES_HUBER_THRESHOLD);

    for (typePantoImagePoint& ImagePoint : KeyFrame->Points.ImagePoints)
    {
        const u64 MapPointID = ImagePoint.MapPointID;
        if(MapPointID != PANTO_ID_NOT_SET)
        {
            typePantoMapPoint& MapPoint = Map.MapPoints[MapPointID];

            if(PT_GetNumObservations(MapPoint) <= 1)
            {
                continue;
            }
            Problem.AddParameterBlock(MapPoint.Point.data(), 4);
            Problem.SetParameterBlockConstant(MapPoint.Point.data());

            ceres::CostFunction* CostFunc =
                OP_ReprojectionError::Create(
                        ImagePoint.Point.x(),
                        ImagePoint.Point.y(),
                        &OPCameraIntrinsics);

            Problem.AddResidualBlock(
                    CostFunc,
                    LossFunc,
                    CameraParameters.Quaternion.coeffs().data(),
                    CameraParameters.tParametrization.data(),
                    MapPoint.Point.data());
        }
    }

#if defined(CONFIG_IMU)
    const Eigen::LLT<Eigen::Matrix<fp64, 15, 15>> LLT(PreviousData.Covariance);
    const Eigen::Matrix<fp64, 15, 15> L = LLT.matrixL();
    const Eigen::Matrix<fp64, 15, 15> SqrtInfo = L.triangularView<Eigen::Lower>().solve(
            Eigen::Matrix<fp64, 15, 15>::Identity());
    ceres::CostFunction* CostFuncIMU = OP_IMUResidual::Create(PreviousData, SqrtInfo, Grav, TBS);

    Problem.AddResidualBlock(CostFuncIMU,
            nullptr,
            PreviousCameraParameters.Quaternion.coeffs().data(),
            PreviousCameraParameters.tParametrization.data(),
            PreviousIMUParameters.Velocity.data(),
            PreviousIMUParameters.GyroBias.data(),
            PreviousIMUParameters.AccelorometerBias.data(),

            CameraParameters.Quaternion.coeffs().data(),
            CameraParameters.tParametrization.data(),
            IMUParameters.Velocity.data(),
            IMUParameters.GyroBias.data(),
            IMUParameters.AccelorometerBias.data()
            );
#endif
}

void __OP_BuildProblemLocal(typeGlobalMap& Map, ceres::Problem& Problem, const typeLocalMap& LocalMap)
{
    /*See https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#manifold
     *Optimizing on manifolds seemingly has many benefits, one of them is that
     *Quaternions are naturally constrained to be normalized, another is that
     *The dimension of the optimization is reduced to its natural size,
     *I am unsure but I think this means that if we have 3DOF but 4 parameters
     *the optimization is reduced to the natural size of optimizing with 3DOF, whatever that means?
    */
    std::unordered_set<u64> FixedKeyFrames;
#if defined(CONFIG_IMU)
    const Eigen::Vector3d Grav = *IMU_GetGravity();
#endif

    // Add all optimized states once.
    for(const u64 KeyFrameID : LocalMap.KeyFrameIDs)
    {
        typeKeyFrame& KeyFrame = Map.KeyFrames[KeyFrameID];

        typeCameraPose& Camera = KeyFrame.Camera.Pose;

        Problem.AddParameterBlock(Camera.Quaternion.coeffs().data(), 4);
        Problem.SetManifold(
                Camera.Quaternion.coeffs().data(),
                new ceres::EigenQuaternionManifold());

        Problem.AddParameterBlock(Camera.tParametrization.data(), 3);

#if defined(CONFIG_IMU)
        typeNavigationState& Nav = KeyFrame.NavigationState;
        Problem.AddParameterBlock(Nav.Velocity.data(), 3);
        Problem.AddParameterBlock(Nav.GyroBias.data(), 3);
        Problem.AddParameterBlock(Nav.AccelorometerBias.data(), 3);
#endif
    }

    u64 countFixed = 0;
#if defined(CONFIG_IMU)
    const u64 AnchorID = LocalMap.IMUAnchor;
    assert(AnchorID != PANTO_ID_NOT_SET);

    typeKeyFrame& Anchor = Map.KeyFrames[AnchorID];

    Problem.AddParameterBlock(Anchor.Camera.Pose.Quaternion.coeffs().data(), 4);
    Problem.SetManifold(Anchor.Camera.Pose.Quaternion.coeffs().data(), new ceres::EigenQuaternionManifold());
    Problem.AddParameterBlock(Anchor.Camera.Pose.tParametrization.data(), 3);

    Problem.AddParameterBlock(Anchor.NavigationState.Velocity.data(), 3);
    Problem.AddParameterBlock(Anchor.NavigationState.GyroBias.data(), 3);
    Problem.AddParameterBlock(Anchor.NavigationState.AccelorometerBias.data(), 3);

    Problem.SetParameterBlockConstant(Anchor.Camera.Pose.Quaternion.coeffs().data());
    Problem.SetParameterBlockConstant(Anchor.Camera.Pose.tParametrization.data());
    Problem.SetParameterBlockConstant(Anchor.NavigationState.Velocity.data());
    Problem.SetParameterBlockConstant(Anchor.NavigationState.GyroBias.data());
    Problem.SetParameterBlockConstant(Anchor.NavigationState.AccelorometerBias.data());
    FixedKeyFrames.insert(AnchorID);
    countFixed++;
#endif

    for(const u64 KeyFrameID : LocalMap.FixedKeyFrameIDs)
    {
        assert(Map.KeyFrames.contains(KeyFrameID));

#if defined(CONFIG_IMU)
        if(KeyFrameID == AnchorID)
        {
            continue;
        }
#endif
        typeCameraPose& CameraParameters = Map.KeyFrames[KeyFrameID].Camera.Pose;

        Problem.AddParameterBlock(CameraParameters.Quaternion.coeffs().data(), 4);
        Problem.SetManifold(CameraParameters.Quaternion.coeffs().data(), new ceres::EigenQuaternionManifold());
        Problem.AddParameterBlock(CameraParameters.tParametrization.data(), 3);

        Problem.SetParameterBlockConstant(CameraParameters.Quaternion.coeffs().data());
        Problem.SetParameterBlockConstant(CameraParameters.tParametrization.data());

        FixedKeyFrames.insert(KeyFrameID);
        countFixed++;
    }

    LG_Log(LogSeverity::DBG, "[__OP_BuildProblemLocal] Has %llu fixed keyframes",  countFixed);


#if !defined(CONFIG_IMU)
    if(FixedKeyFrames.size() < 2)
    {
        for(const u64 KeyFrameID : LocalMap.KeyFrameIDs)
        {
            if(FixedKeyFrames.contains(KeyFrameID))
            {
                continue;
            }
            typeCameraPose& CameraParameters = Map.KeyFrames[KeyFrameID].Camera.Pose;

            Problem.SetParameterBlockConstant(CameraParameters.Quaternion.coeffs().data());
            Problem.SetParameterBlockConstant(CameraParameters.tParametrization.data());
            FixedKeyFrames.insert(KeyFrameID);

            if(FixedKeyFrames.size() == 2)
            {
                break;
            }
        }
    }
#endif // CONFIG_IMU

    std::unordered_set<u64> LocalMapPoints;

    for(const u64& LocalMapPointID : LocalMap.MapPointIDs)
    {
        assert(Map.MapPoints.contains(LocalMapPointID));

        if(PT_GetNumObservations(Map.MapPoints[LocalMapPointID]) <= 1)
        {
            continue;
        }

        typePantoMapPoint& MapPoint = Map.MapPoints[LocalMapPointID];
        Problem.AddParameterBlock( MapPoint.Point.data(), 4);
        Problem.SetManifold(MapPoint.Point.data(), new ceres::SphereManifold<4>());
        LocalMapPoints.insert(LocalMapPointID);
    }

    ceres::LossFunction* lossfunc = new ceres::HuberLoss( CERES_HUBER_THRESHOLD);

    for(const u64& KeyFrameID : LocalMap.KeyFrameIDs)
    {
        typeKeyFrame& KeyFrame = Map.KeyFrames[KeyFrameID];

        typeCameraPose& CameraParameters = KeyFrame.Camera.Pose;
#if defined(CONFIG_IMU)
        typeNavigationState& IMUParameters = KeyFrame.NavigationState;
#endif

        for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID != PANTO_ID_NOT_SET && LocalMapPoints.contains(MapPointID))
            {
                assert(Map.MapPoints.contains(MapPointID));
                if(PT_GetNumObservations(Map.MapPoints[MapPointID]) <= 1)
                {
                    continue;
                }

                const fp64 PointX = ImagePoint.Point.x();
                const fp64 PointY = ImagePoint.Point.y();

                ceres::CostFunction* costfunc = OP_ReprojectionError::Create( PointX, PointY,
                            &OPCameraIntrinsics);


                Problem.AddResidualBlock(costfunc, lossfunc,
                        CameraParameters.Quaternion.coeffs().data(),
                        CameraParameters.tParametrization.data(),
                        Map.MapPoints[MapPointID].Point.data());
            }
        }

        if(KeyFrameID == 0)
        {
            continue;
        }

#if defined(CONFIG_IMU)
        typeKeyFrame& PreviousKeyFrame = Map.KeyFrames[KeyFrame.PreviousKFID];

        typeCameraPose& PreviousCameraParameters = PreviousKeyFrame.Camera.Pose;
        typeNavigationState& PreviousIMUParameters = PreviousKeyFrame.NavigationState;
        typePreIntegrationData& PreviousData = KeyFrame.PreIntegrationData;

        const Eigen::LLT<Eigen::Matrix<fp64, 15, 15>> LLT(PreviousData.Covariance);
        const Eigen::Matrix<fp64, 15, 15> L = LLT.matrixL();
        Eigen::Matrix<fp64, 15, 15> SqrtInfo = L.triangularView<Eigen::Lower>().solve(
                Eigen::Matrix<fp64, 15, 15>::Identity());

        ceres::LossFunction* IMULoss = nullptr;
        if(PreviousKeyFrame.ID == AnchorID)
        {
            // Match ORB-SLAM3's treatment of the inertial edge crossing the
            // fixed-window boundary: information *= 1e-2, therefore the
            // square-root information used here must be multiplied by 0.1.
            SqrtInfo *= 0.1;
            IMULoss = new ceres::HuberLoss(std::sqrt(16.92));
        }

        ceres::CostFunction* CostFuncIMU = OP_IMUResidual::Create(PreviousData, SqrtInfo, Grav, TBS);

        Problem.AddResidualBlock(CostFuncIMU,
                IMULoss,
                PreviousCameraParameters.Quaternion.coeffs().data(),
                PreviousCameraParameters.tParametrization.data(),
                PreviousIMUParameters.Velocity.data(),
                PreviousIMUParameters.GyroBias.data(),
                PreviousIMUParameters.AccelorometerBias.data(),

                CameraParameters.Quaternion.coeffs().data(),
                CameraParameters.tParametrization.data(),
                IMUParameters.Velocity.data(),
                IMUParameters.GyroBias.data(),
                IMUParameters.AccelorometerBias.data()
            );
#endif
    }

    for(const u64& KeyFrameID : LocalMap.FixedKeyFrameIDs)
    {
        typeKeyFrame& KeyFrame = Map.KeyFrames[KeyFrameID];

        typeCameraPose& CameraParameters = KeyFrame.Camera.Pose;

        for(typePantoImagePoint& ImagePoint : KeyFrame.Points.ImagePoints)
        {
            const u64 MapPointID = ImagePoint.MapPointID;

            if(MapPointID != PANTO_ID_NOT_SET && LocalMapPoints.contains(MapPointID))
            {
                assert(Map.MapPoints.contains(MapPointID));

                if(PT_GetNumObservations(Map.MapPoints[MapPointID]) <= 1)
                {
                    continue;
                }

                const fp64 PointX = ImagePoint.Point.x();

                const fp64 PointY = ImagePoint.Point.y();

                ceres::CostFunction* costfunc = OP_ReprojectionError::Create( PointX, PointY,
                            &OPCameraIntrinsics);

                Problem.AddResidualBlock(costfunc, lossfunc,
                        CameraParameters.Quaternion.coeffs().data(),
                        CameraParameters.tParametrization.data(),
                        Map.MapPoints[MapPointID].Point.data());
            }
        }
    }
}

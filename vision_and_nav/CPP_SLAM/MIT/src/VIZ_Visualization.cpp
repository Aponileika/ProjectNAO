#include "VIZ_Visualization.hpp"
#include "VIZPriv_Visualization.hpp"

static std::vector<cv::Mat> VIZPriv_KeyFrameImages;

static pid_t VIZPriv_ViewerPID = -1;

void VIZ_InitVisualization(void)
{
    VIZPriv_KeyFrameImages.clear();

    const std::string SparsePath =
        std::string(PANTO_COLMAP_PATH) + "/sparse";

    const std::string SnapshotPath =
        SparsePath + "/snapshots";

    const std::string LatestPath =
        SparsePath + "/latest.txt";

    const std::string TemporaryLatestPath =
        SparsePath + "/latest.txt.tmp";

    if(std::filesystem::exists(SnapshotPath))
    {
        std::filesystem::remove_all(SnapshotPath);
    }

    if(std::filesystem::exists(LatestPath))
    {
        std::filesystem::remove(LatestPath);
    }

    if(std::filesystem::exists(TemporaryLatestPath))
    {
        std::filesystem::remove(TemporaryLatestPath);
    }

    std::filesystem::create_directories(SnapshotPath);

    const pid_t PID = fork();

    if(PID < 0)
    {
        LG_Log(LogSeverity::ERROR, "[VIZ_StartViewer] Failed to fork viewer process\n");
        std::exit(1);
    }

    if(PID == 0)
    {
        execl(
                PANTO_PATH_TO_PYTHON_INTERPRETER,
                PANTO_PATH_TO_PYTHON_INTERPRETER,
                PANTO_COLMAP_PYTHON_SCRIPT_PATH,
                PANTO_COLMAP_PATH,
                static_cast<char*>(nullptr));

        _exit(1);
    }

    VIZPriv_ViewerPID = PID;
    std::atexit(VIZ_DestroyVisualization);

    LG_Log(LogSeverity::DBG, "[VIZ_StartViewer] Started viewer process PID = %d\n",
            static_cast<i32>(PID));
}

void VIZ_DestroyVisualization(void)
{
    if(VIZPriv_ViewerPID <= 0)
    {
        return;
    }

    kill(VIZPriv_ViewerPID, SIGTERM);

    waitpid(
            VIZPriv_ViewerPID,
            nullptr,
            0);

    LG_Log(LogSeverity::DBG,
            "[VIZ_DestroyVisualization] Stopped viewer process PID = %d\n",
            static_cast<i32>(VIZPriv_ViewerPID));

    VIZPriv_ViewerPID = -1;
}

void VIZ_WriteColmap(const typeGlobalMap& GlobalMap)
{
    static u64 SnapshotID = 0;

    const std::string SnapshotPath =
        std::string(PANTO_COLMAP_PATH) + "/sparse/snapshots/" + std::to_string(SnapshotID);

    std::filesystem::create_directories(SnapshotPath);

    VIZPriv_LoadKeyFrameImages(GlobalMap.KeyFrames);

    VIZPriv_WriteCameras(GlobalMap.KeyFrames, SnapshotPath);
    VIZPriv_WriteImages(GlobalMap.KeyFrames, SnapshotPath);
    VIZPriv_WritePoints(GlobalMap, SnapshotPath);

    LG_Log(LogSeverity::DBG,
            "[VIZ_WriteColmap] Publishing snapshot %llu from path %s\n",
            static_cast<unsigned long long>(SnapshotID),
            SnapshotPath.c_str());

    VIZPriv_PublishSnapshot(SnapshotID);

    SnapshotID++;
}

void VIZPriv_WriteCameras(const typePantoVector<typeKeyFrame>& KeyFrames, const std::string& SnapshotPath)
{
    const std::string CameraPath =
        SnapshotPath + "/cameras.bin";

    FILE* fp =
        fopen(CameraPath.c_str(), "wb");

    if(fp == nullptr)
    {
        LG_Log(LogSeverity::DBG,
                "[VIZPriv_WriteCameras] Failed to open %s\n",
                CameraPath.c_str());

        return;
    }

    const u64 NumCameras =
        static_cast<u64>(KeyFrames.active_size());

    fwrite(
            &NumCameras,
            sizeof(u64),
            1,
            fp);

    u64 NumWritten = 0;

    for(const typeKeyFrame& KeyFrame : KeyFrames)
    {
        const i32 CameraID =
            static_cast<i32>(KeyFrame.ID + 1);

        const i32 ModelID =
            PANTO_CAMERA_MODEL_ID;

        const typeCameraIntrinsics* Intrinsics =
            KeyFrame.Pose.Intrinsics;

        assert(Intrinsics != nullptr);

        const u64 Width =
            PANTO_IMAGE_WIDTH;

        const u64 Height =
            PANTO_IMAGE_HEIGHT;

        const fp64 Parameters[4] =
        {
            Intrinsics->K(0, 0),
            Intrinsics->K(1, 1),
            Intrinsics->K(0, 2),
            Intrinsics->K(1, 2)
        };

        fwrite(&CameraID, sizeof(i32), 1, fp);
        fwrite(&ModelID, sizeof(i32), 1, fp);
        fwrite(&Width, sizeof(u64), 1, fp);
        fwrite(&Height, sizeof(u64), 1, fp);
        fwrite(Parameters, sizeof(fp64), 4, fp);

        NumWritten++;
    }

    assert(NumWritten == NumCameras);

    fclose(fp);

    LG_Log(LogSeverity::DBG,
            "[VIZPriv_WriteCameras] Header = %llu, Written = %llu\n",
            static_cast<unsigned long long>(NumCameras),
            static_cast<unsigned long long>(NumWritten));
}

void VIZPriv_WriteImages(const typePantoVector<typeKeyFrame>& KeyFrames, const std::string& SnapshotPath)
{
    const std::string ImagePath =
        SnapshotPath + "/images.bin";

    static_assert(sizeof(u64) == 8);
    static_assert(sizeof(fp64) == 8);

    FILE* fp =
        fopen(ImagePath.c_str(), "wb");

    if(fp == nullptr)
    {
        LG_Log(LogSeverity::DBG,
                "[VIZPriv_WriteImages] Failed to open %s\n",
                ImagePath.c_str());

        return;
    }

    const u64 NumImages =
        static_cast<u64>(KeyFrames.active_size());

    fwrite(
            &NumImages,
            sizeof(u64),
            1,
            fp);

    for(const typeKeyFrame& KeyFrame : KeyFrames)
    {
        const i32 ImageID =
            static_cast<i32>(KeyFrame.ID + 1);

        const i32 CameraID =
            static_cast<i32>(KeyFrame.ID + 1);

        fwrite(
                &ImageID,
                sizeof(i32),
                1,
                fp);

        const Eigen::Matrix3d& Rcw =
            KeyFrame.Pose.Pose.R;

        const Eigen::Vector3d& tcw =
            KeyFrame.Pose.Pose.t;

        Eigen::Quaterniond Quaternion(Rcw);

        Quaternion.normalize();

        const fp64 COLMAPQuaternion[4] =
        {
            Quaternion.w(),
            Quaternion.x(),
            Quaternion.y(),
            Quaternion.z()
        };

        fwrite(
                COLMAPQuaternion,
                sizeof(fp64),
                4,
                fp);

        fwrite(
                tcw.data(),
                sizeof(fp64),
                3,
                fp);

        fwrite(
                &CameraID,
                sizeof(i32),
                1,
                fp);

        const std::filesystem::path CurrentImagePath(
                KeyFrame.ImagePath);

        const std::string ImageName =
            CurrentImagePath.filename().string();

        fwrite(
                ImageName.c_str(),
                sizeof(char),
                ImageName.size() + 1,
                fp);

        const u64 NumImagePoints =
            static_cast<u64>(
                    KeyFrame.Points.ImagePoints.active_size());

        fwrite(
                &NumImagePoints,
                sizeof(u64),
                1,
                fp);

        LG_Log(
                LogSeverity::DBG,
                "[VIZPriv_WriteImages] ImageID = %d, CameraID = %d, Name = '%s', NumPoints2D = %llu\n",
                ImageID,
                CameraID,
                ImageName.c_str(),
                static_cast<unsigned long long>(NumImagePoints));

        for(const typePantoImagePoint& ImagePoint :
            KeyFrame.Points.ImagePoints)
        {
            const fp64 Point2D[2] =
            {
                ImagePoint.Point.x(),
                ImagePoint.Point.y()
            };

            fwrite(
                    Point2D,
                    sizeof(fp64),
                    2,
                    fp);

            i64 Point3DID = -1;

            if(ImagePoint.MapPointID != PANTO_ID_NOT_SET)
            {
                Point3DID =
                    static_cast<i64>(
                            ImagePoint.MapPointID + 1);
            }

            fwrite(
                    &Point3DID,
                    sizeof(i64),
                    1,
                    fp);
        }
    }

    fclose(fp);

    LG_Log(
            LogSeverity::DBG,
            "[VIZPriv_WriteImages] Wrote %llu images to %s\n",
            static_cast<unsigned long long>(NumImages),
            ImagePath.c_str());
}

void VIZPriv_WritePoints(const typeGlobalMap& GlobalMap, const std::string& SnapshotPath)
{
    const std::string PointPath =
        SnapshotPath + "/points3D.bin";

    FILE* fp =
        fopen(PointPath.c_str(), "wb");

    if(fp == nullptr)
    {
        LG_Log(
                LogSeverity::DBG,
                "[VIZPriv_WritePoints] Failed to open %s\n",
                PointPath.c_str());

        return;
    }

    const u64 NumPoints =
        static_cast<u64>(
                GlobalMap.MapPoints.active_size());

    fwrite(
            &NumPoints,
            sizeof(u64),
            1,
            fp);

    for(const typePantoMapPoint& MapPoint :
        GlobalMap.MapPoints)
    {
        const u64 Point3DID =
            MapPoint.ID + 1;

        fwrite(
                &Point3DID,
                sizeof(u64),
                1,
                fp);

        const Eigen::Vector3d Point =
            PROJ_Homog2Cart(MapPoint.Point);

        fwrite(
                Point.data(),
                sizeof(fp64),
                3,
                fp);

        u8 RGB[3] =
        {
            255,
            255,
            255
        };

        assert(
                MapPoint.KeyFrameIDs.size() ==
                MapPoint.ImagePointIDs.size());

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            assert(
                    MapPoint.KeyFrameIDs.contains(i) ==
                    MapPoint.ImagePointIDs.contains(i));
        }

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            if(!MapPoint.KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID =
                MapPoint.KeyFrameIDs[i];

            const u64 ImagePointID =
                MapPoint.ImagePointIDs[i];

            if(!GlobalMap.KeyFrames.contains(KeyFrameID))
            {
                continue;
            }

            if(KeyFrameID >= VIZPriv_KeyFrameImages.size())
            {
                continue;
            }

            const typeKeyFrame& KeyFrame =
                GlobalMap.KeyFrames[KeyFrameID];

            if(!KeyFrame.Points.ImagePoints.contains(ImagePointID))
            {
                continue;
            }

            const cv::Mat& Image =
                VIZPriv_KeyFrameImages[KeyFrameID];

            if(Image.empty())
            {
                continue;
            }

            const typePantoImagePoint& ImagePoint =
                KeyFrame.Points.ImagePoints[ImagePointID];

            const i32 X =
                static_cast<i32>(
                        std::round(
                                ImagePoint.Point.x()));

            const i32 Y =
                static_cast<i32>(
                        std::round(
                                ImagePoint.Point.y()));

            if(X < 0 ||
               Y < 0 ||
               X >= Image.cols ||
               Y >= Image.rows)
            {
                continue;
            }

            const cv::Vec3b BGR =
                Image.at<cv::Vec3b>(Y, X);

            RGB[0] = BGR[2];
            RGB[1] = BGR[1];
            RGB[2] = BGR[0];

            break;
        }

        fwrite(
                RGB,
                sizeof(u8),
                3,
                fp);

        const fp64 Error =
            0.0;

        fwrite(
                &Error,
                sizeof(fp64),
                1,
                fp);

        u64 TrackLength = 0;

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            assert(
                    MapPoint.KeyFrameIDs.contains(i) ==
                    MapPoint.ImagePointIDs.contains(i));

            if(!MapPoint.KeyFrameIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID =
                MapPoint.KeyFrameIDs[i];

            const u64 ImagePointID =
                MapPoint.ImagePointIDs[i];

            if(!GlobalMap.KeyFrames.contains(KeyFrameID))
            {
                continue;
            }

            const typeKeyFrame& KeyFrame =
                GlobalMap.KeyFrames[KeyFrameID];

            if(!KeyFrame.Points.ImagePoints.contains(ImagePointID))
            {
                continue;
            }

            TrackLength++;
        }

        fwrite(
                &TrackLength,
                sizeof(u64),
                1,
                fp);

        for(std::size_t i{}; i < MapPoint.KeyFrameIDs.size(); i++)
        {
            if(!MapPoint.KeyFrameIDs.contains(i) ||
               !MapPoint.ImagePointIDs.contains(i))
            {
                continue;
            }

            const u64 KeyFrameID =
                MapPoint.KeyFrameIDs[i];

            const u64 ImagePointID =
                MapPoint.ImagePointIDs[i];

            if(!GlobalMap.KeyFrames.contains(KeyFrameID))
            {
                continue;
            }

            const typeKeyFrame& KeyFrame =
                GlobalMap.KeyFrames[KeyFrameID];

            if(!KeyFrame.Points.ImagePoints.contains(ImagePointID))
            {
                continue;
            }

            const i32 ImageID =
                static_cast<i32>(
                        KeyFrameID + 1);

            const i32 Point2DIdx =
                static_cast<i32>(
                        ImagePointID);

            fwrite(
                    &ImageID,
                    sizeof(i32),
                    1,
                    fp);

            fwrite(
                    &Point2DIdx,
                    sizeof(i32),
                    1,
                    fp);
        }
    }

    fclose(fp);

    LG_Log(
            LogSeverity::DBG,
            "[VIZPriv_WritePoints] Wrote %llu points to %s\n",
            static_cast<unsigned long long>(NumPoints),
            PointPath.c_str());
}

void VIZPriv_PublishSnapshot(const u64& SnapshotID)
{
    const std::string TemporaryPath =
        std::string(PANTO_COLMAP_PATH) + "/sparse/latest.txt.tmp";

    const std::string LatestPath =
        std::string(PANTO_COLMAP_PATH) + "/sparse/latest.txt";

    FILE* fp =
        fopen(TemporaryPath.c_str(), "w");

    if(fp == nullptr)
    {
        LG_Log(
                LogSeverity::DBG,
                "[VIZPriv_PublishSnapshot] Failed to open temporary latest file\n");

        return;
    }

    fprintf(
            fp,
            "%llu\n",
            static_cast<unsigned long long>(SnapshotID));

    fclose(fp);

    std::filesystem::rename(
            TemporaryPath,
            LatestPath);
}

void VIZPriv_LoadKeyFrameImages(const typePantoVector<typeKeyFrame>& KeyFrames)
{
    if(VIZPriv_KeyFrameImages.size() <
       KeyFrames.size())
    {
        VIZPriv_KeyFrameImages.resize(
                KeyFrames.size());
    }

    for(const typeKeyFrame& KeyFrame :
        KeyFrames)
    {
        assert(
                KeyFrame.ID <
                VIZPriv_KeyFrameImages.size());

        if(!VIZPriv_KeyFrameImages[
                KeyFrame.ID].empty())
        {
            continue;
        }

        VIZPriv_KeyFrameImages[
                KeyFrame.ID] =
            cv::imread(
                    KeyFrame.ImagePath,
                    cv::IMREAD_COLOR);

        if(VIZPriv_KeyFrameImages[
                KeyFrame.ID].empty())
        {
            LG_Log(
                    LogSeverity::DBG,
                    "[VIZPriv_LoadKeyFrameImages] Failed to read %s\n",
                    KeyFrame.ImagePath.c_str());
        }
    }
}

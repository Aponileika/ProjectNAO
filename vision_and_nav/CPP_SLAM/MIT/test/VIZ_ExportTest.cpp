#include "VIZ_Visualization.hpp"
#include <fstream>
#include <iostream>

void LG_Log(LogSeverity, const char*, ...) {}
#include "../src/VIZ_Visualization.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

int main()
{
    namespace fs = std::filesystem;
    const fs::path Workspace = fs::current_path();
    const fs::path Directory = fs::temp_directory_path() /
        ("cpp_slam_viz_" + std::to_string(PantoClock::now().time_since_epoch().count()));
    fs::create_directories(Directory);
    fs::current_path(Directory);
    const std::vector<Eigen::Vector3d> Trajectory{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    VIZ_SetGroundTruth(Trajectory, {0.25, 0.5});
    const fs::path OutputRoot = PANTO_COLMAP_PATH;
    std::ifstream Positions(OutputRoot / "sparse/ground_truth.bin", std::ios::binary);
    u64 Count = 0;
    fp64 Coordinates[6]{};
    Positions.read(reinterpret_cast<char*>(&Count), sizeof(Count));
    Positions.read(reinterpret_cast<char*>(Coordinates), sizeof(Coordinates));
    Require(Positions.good() && Count == 2 && Coordinates[0] == 1.0 && Coordinates[5] == 6.0,
            "trajectory format preserved and missing parent created");
    Require(fs::file_size(OutputRoot / "sparse/ground_truth_timestamps.bin") == sizeof(u64) + 2 * sizeof(fp64),
            "timestamp export");

    fs::create_directory("not-a-file.bin");
    Require(!VIZPriv_WriteTrajectoryFile(Trajectory, "not-a-file.bin"), "trajectory fopen failure handled");
    Require(!VIZPriv_WriteTimeStampFile({0.25}, "not-a-file.bin"), "timestamp fopen failure handled");
    std::ofstream("blocked-parent") << "file";
    Require(!VIZPriv_WriteTrajectoryFile(Trajectory, "blocked-parent/tracking.bin"), "parent creation failure handled");

    fs::create_directory("old-output");
    std::ofstream("old-output/saved.txt") << "preserve";
    fs::permissions("old-output", fs::perms::owner_read | fs::perms::owner_exec);
    Require(!VIZPriv_ResetOutputDirectory("old-output"), "locked output reports failure without aborting");
    Require(fs::is_regular_file("old-output/saved.txt"), "locked output preserved");
    fs::permissions("old-output", fs::perms::owner_all);
    Require(VIZPriv_ResetOutputDirectory("old-output"), "writable output reset");
    std::ofstream Probe("old-output/new.txt");
    Require(Probe.good(), "replacement output is writable");
    Probe.close();
    fs::create_directories(OutputRoot / "sparse/snapshots/0/tracking.bin");
    typeGlobalMap EmptyMap{};
    VIZ_WriteRealtime(EmptyMap, Trajectory);
    Require(!fs::exists(OutputRoot / "sparse/latest.txt") && VIZPriv_SnapshotID == 0,
            "failed snapshot is not published");
    fs::remove(OutputRoot / "sparse/snapshots/0/tracking.bin");
    VIZ_WriteRealtime(EmptyMap, Trajectory);
    Require(fs::is_regular_file(OutputRoot / "sparse/snapshots/0/tracking.bin") && VIZPriv_SnapshotID == 1,
            "valid snapshot written after export failure");
    std::ifstream Latest(OutputRoot / "sparse/latest.txt");
    u64 Snapshot = 99;
    Latest >> Snapshot;
    Require(Latest.good() && Snapshot == 0, "latest pointer publishes completed snapshot");

    typeCameraIntrinsics Intrinsics{};
    Intrinsics.K.setIdentity();
    typeKeyFrame Frame{};
    Frame.ID = 7;
    Frame.Camera.Intrinsics = &Intrinsics;
    Frame.Camera.Pose.R.setIdentity();
    Frame.Camera.Pose.t.setZero();
    Frame.ImagePath = "7.png";
    typeGlobalMap DisplayMap{};
    DisplayMap.KeyFrames.push_back(Frame);
    VIZ_WriteRealtime(DisplayMap, Trajectory, true);
    const auto SnapshotRoot = OutputRoot / "sparse/snapshots";
    std::string Mode;
    std::ifstream(SnapshotRoot / "1/keyframes_mode.txt") >> Mode;
    Require(Mode == "append", "normal live snapshot declares append semantics");
    std::ifstream Images(SnapshotRoot / "1/images.bin", std::ios::binary);
    u64 ImageCount = 0;
    i32 ImageID = 0;
    Images.read(reinterpret_cast<char*>(&ImageCount), sizeof(ImageCount));
    Images.read(reinterpret_cast<char*>(&ImageID), sizeof(ImageID));
    Require(Images.good() && ImageCount == 1 && ImageID == 8, "delta contains one KF with its original ID");
    Frame.ID = 8;
    Frame.ImagePath = "8.png";
    DisplayMap.KeyFrames.push_back(Frame);
    VIZ_WriteRealtime(DisplayMap, Trajectory);
    std::ifstream(SnapshotRoot / "2/keyframes_mode.txt") >> Mode;
    Require(Mode == "replace", "final/reset live snapshot replaces old poses");
    std::ifstream FinalImages(SnapshotRoot / "2/images.bin", std::ios::binary);
    FinalImages.read(reinterpret_cast<char*>(&ImageCount), sizeof(ImageCount));
    Require(FinalImages.good() && ImageCount == 2, "full replacement still contains all KFs");

    fs::current_path(Workspace);
    fs::remove_all(Directory);
    std::cout << "Visualization export tests passed\n";
}

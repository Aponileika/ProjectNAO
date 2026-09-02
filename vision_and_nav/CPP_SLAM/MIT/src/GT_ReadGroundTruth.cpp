#include "GT_ReadGroundTruth.hpp"

static std::string GTPriv_GetGroundTruthPath(void)
{
    return std::string(PANTO_DATASET_BASE_PATH) +
        std::string(panto_dataset_path) +
        std::string(panto_gt_path) +
        "/data.csv";
}

static bool GTPriv_ReadMeasurement(
        FILE* GroundTruthFile,
        typeGroundTruth& GroundTruth)
{
    char Buffer[1024];

    if(std::fgets(Buffer, sizeof(Buffer), GroundTruthFile) == nullptr)
    {
        return false;
    }

    unsigned long long TimeStampNS;

    fp64 Px;
    fp64 Py;
    fp64 Pz;

    fp64 Qw;
    fp64 Qx;
    fp64 Qy;
    fp64 Qz;

    fp64 Vx;
    fp64 Vy;
    fp64 Vz;

    fp64 Bgx;
    fp64 Bgy;
    fp64 Bgz;

    fp64 Bax;
    fp64 Bay;
    fp64 Baz;

    const int Parsed = std::sscanf(
        Buffer,
        "%llu,"
        "%lf,%lf,%lf,"
        "%lf,%lf,%lf,%lf,"
        "%lf,%lf,%lf,"
        "%lf,%lf,%lf,"
        "%lf,%lf,%lf",
        &TimeStampNS,
        &Px, &Py, &Pz,
        &Qw, &Qx, &Qy, &Qz,
        &Vx, &Vy, &Vz,
        &Bgx, &Bgy, &Bgz,
        &Bax, &Bay, &Baz);

    assert(Parsed == 17);

    GroundTruth =
    {
        .TimeStamp = static_cast<fp64>(TimeStampNS) * 1e-9,

        .Position = Eigen::Vector3d(Px, Py, Pz),

        .Orientation = Eigen::Quaterniond(Qw, Qx, Qy, Qz),

        .Velocity = Eigen::Vector3d(Vx, Vy, Vz),

        .GyroBias = Eigen::Vector3d(Bgx, Bgy, Bgz),

        .AccelBias = Eigen::Vector3d(Bax, Bay, Baz)
    };

    return true;
}

typeGroundTruth GT_GetMeasurement(void)
{
    static FILE* GroundTruthFile = nullptr;

    if(GroundTruthFile == nullptr)
    {
        GroundTruthFile = std::fopen(
            GTPriv_GetGroundTruthPath().c_str(),
            "r");

        assert(GroundTruthFile != nullptr);

        char Header[1024];
        assert(std::fgets(
            Header,
            sizeof(Header),
            GroundTruthFile) != nullptr);
    }

    typeGroundTruth GroundTruth{};

    return GTPriv_ReadMeasurement(GroundTruthFile, GroundTruth)
        ? GroundTruth
        : typeGroundTruth{};
}

std::vector<typeGroundTruth> GT_GetAllMeasurements(void)
{
    FILE* GroundTruthFile = std::fopen(
            GTPriv_GetGroundTruthPath().c_str(),
            "r");

    assert(GroundTruthFile != nullptr);

    char Header[1024];
    assert(std::fgets(
        Header,
        sizeof(Header),
        GroundTruthFile) != nullptr);

    std::vector<typeGroundTruth> Measurements;
    typeGroundTruth GroundTruth{};

    while(GTPriv_ReadMeasurement(GroundTruthFile, GroundTruth))
    {
        Measurements.push_back(GroundTruth);
    }

    std::fclose(GroundTruthFile);
    return Measurements;
}

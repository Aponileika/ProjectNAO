#include "TH_Threading.hpp"
#include "CM_Camera.hpp"
#include <cblas.h>
#include <iostream>
#include <thread>
#include <vector>

void LG_Log(LogSeverity, const char*, ...) {}
#include "../src/TH_Threading.cpp"
#include "../src/CM_Camera.cpp"

static void Require(bool Condition, const char* Message)
{
    if(!Condition)
    {
        std::cerr << "FAIL: " << Message << '\n';
        std::exit(1);
    }
}

// Exercise the same pre-main calibration path as OP's static intrinsics.
static const bool EarlyCalibration = []()
{
    CM_SetIntrinsics();
    Require(openblas_get_num_threads() == BLAS_NUM_THREADS, "OpenBLAS configured before calibration");
    if(OPENCV_NUM_THREADS > 0)
    {
        Require(cv::getNumThreads() == OPENCV_NUM_THREADS, "OpenCV configured before calibration");
    }
    return true;
}();

static void CheckBLAS()
{
#if defined(__APPLE__)
    if(openblas_get_parallel() == 2)
    {
        Require(omp_get_max_threads() == BLAS_NUM_THREADS, "OpenMP count matches BLAS config");
    }
#endif
    constexpr int Size = 256;
    std::vector<double> A(Size * Size, 1.0);
    std::vector<double> B(Size * Size, 1.0);
    std::vector<double> C(Size * Size, 0.0);
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, Size, Size, Size,
            1.0, A.data(), Size, B.data(), Size, 0.0, C.data(), Size);
    Require(C.front() == Size && C.back() == Size, "BLAS remains operational");
    Require(openblas_get_num_threads() == BLAS_NUM_THREADS, "OpenBLAS count retained after math work");
}

static void CheckWorker(const char* Name)
{
    TH_SetUserInitiatedQoS(Name);
#if defined(__APPLE__)
    qos_class_t Class = QOS_CLASS_UNSPECIFIED;
    int RelativePriority = 0;
    Require(pthread_get_qos_class_np(pthread_self(), &Class, &RelativePriority) == 0 &&
            Class == QOS_CLASS_USER_INITIATED && RelativePriority == 0, "user-initiated worker QoS");
#endif
#if defined(TH_HAS_ACCELERATE_THREADING)
    if(__builtin_available(macOS 15.0, *))
    {
        Require(BLASGetThreading() == (ACCELERATE_BLAS_SINGLE_THREADED ?
                    BLAS_THREADING_SINGLE_THREADED : BLAS_THREADING_MULTI_THREADED), "worker Accelerate threading policy");
    }
#endif
    CheckBLAS();
}

int main()
{
    (void)EarlyCalibration;
    TH_InitializeLibraries(); // repeated calls must not reconfigure live pools
    std::thread Tracking(CheckWorker, "tracking test");
    std::thread Mapping(CheckWorker, "mapping test");
    Tracking.join();
    Mapping.join();

    CheckBLAS();
    // Library-created native threads must also inherit the configured default.
    std::thread LibraryWorker(CheckBLAS);
    LibraryWorker.join();
    std::cout << "Thread controls passed: OpenBLAS=" << openblas_get_num_threads()
        << ", OpenCV=" << cv::getNumThreads() << ", Accelerate single=" << ACCELERATE_BLAS_SINGLE_THREADED
        << ", worker QoS=user initiated\n";
}

#include "TH_Threading.hpp"
#include "Config.hpp"
#include <cblas.h>
#include <opencv2/core/utility.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(__APPLE__)
#include <omp.h>
#include <pthread/qos.h>
#if __has_include(<vecLib/thread_api.h>)
#include <vecLib/thread_api.h>
#define TH_HAS_ACCELERATE_THREADING 1
#endif
#endif

static_assert(BLAS_NUM_THREADS > 0, "BLAS_NUM_THREADS must be positive");
static_assert(OPENCV_NUM_THREADS == -1 || OPENCV_NUM_THREADS > 0,
        "OPENCV_NUM_THREADS must be -1 or positive");

void TH_ConfigureCurrentThread()
{
#if defined(__APPLE__)
    if(openblas_get_parallel() == 2)
    {
        omp_set_num_threads(BLAS_NUM_THREADS);
    }
#endif
#if defined(TH_HAS_ACCELERATE_THREADING)
    if(__builtin_available(macOS 15.0, *))
    {
        const BLAS_THREADING Policy = ACCELERATE_BLAS_SINGLE_THREADED ?
            BLAS_THREADING_SINGLE_THREADED : BLAS_THREADING_MULTI_THREADED;
        if(BLASSetThreading(Policy) != 0)
        {
            std::fprintf(stderr, "[Threading] Accelerate rejected the BLAS threading policy\n");
        }
    }
#endif
}

void TH_InitializeLibraries()
{
    static const bool Initialized = []()
    {
        // Set this before calibration's first Accelerate call: the library can
        // cache the environment setting. The runtime policy also covers each
        // tracking/mapping caller on macOS 15 and later.
#if defined(__APPLE__)
        if(ACCELERATE_BLAS_SINGLE_THREADED && setenv("VECLIB_MAXIMUM_THREADS", "1", 1) != 0)
        {
            std::perror("[Threading] Cannot set VECLIB_MAXIMUM_THREADS");
        }
#endif
#if defined(__APPLE__)
        if(openblas_get_parallel() == 2)
        {
            // Homebrew's OpenBLAS uses LLVM OpenMP. Its BLAS setter alone can
            // be overridden on the first matrix operation. Set the runtime
            // default too, including native threads later created by Ceres.
            // This also caps other users of the same OpenMP runtime.
            char Defaults[64];
            std::snprintf(Defaults, sizeof(Defaults), "OMP_NUM_THREADS=%d", BLAS_NUM_THREADS);
            kmp_set_defaults(Defaults);
        }
#endif
        openblas_set_num_threads(BLAS_NUM_THREADS);
        // OpenCV's setter is global and must not be called from parallel work.
        cv::setNumThreads(OPENCV_NUM_THREADS);
        TH_ConfigureCurrentThread();
        return true;
    }();
    (void)Initialized;
}

void TH_SetUserInitiatedQoS(const char* ThreadName)
{
#if defined(__APPLE__)
    const int Error = pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
    if(Error != 0)
    {
        std::fprintf(stderr, "[Threading] Cannot set %s QoS: %s\n", ThreadName, std::strerror(Error));
    }
#else
    (void)ThreadName;
#endif
    TH_ConfigureCurrentThread();
}

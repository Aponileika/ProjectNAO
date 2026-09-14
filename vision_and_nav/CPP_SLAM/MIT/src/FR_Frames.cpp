#include "../include/FR_Frames.hpp"
#include "Config.hpp"
#include "EP_CorrespondingPoints.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>

namespace 
{
    struct capture
    {
        cv::VideoCapture cap;
        bool isInit;
        i32 FrameIndex;
    };
    struct capture cap;
}

namespace 
{
    struct typeDecodedDataSetFrame
    {
        cv::Mat Gray;
#if defined(CONFIG_STEREO)
        cv::Mat RightGray;
#endif
        DescRet Descriptors;
        fp64 TimeStamp = PANTO_TIMESTAMP_NOT_SET;
        std::string SourcePath;
    };

    struct dataset_read 
    {
        std::string ImagePath;
        std::ifstream TimeStampFile;
#if defined(CONFIG_STEREO)
        std::string RightImagePath;
        std::ifstream RightTimeStampFile;
#endif
        u64 OutputFrameIndex;
        std::string BufferedFramePath;
#if defined(CONFIG_STEREO)
        std::string BufferedRightFramePath;
#endif
        fp64 BufferedTimeStamp;
        bool HasBufferedFrame;
        std::deque<typeDecodedDataSetFrame> PreloadedFrames;
        std::mutex PreloadMutex;
        std::condition_variable PreloadNotEmpty;
        std::condition_variable_any PreloadNotFull;
        std::jthread PreloadThread;
        bool PreloadEndOfStream = false;
    };
    struct dataset_read reader;
}

namespace 
{
    const PantoClock::time_point StartTime = PantoClock::now();
}

typePantoFrame __FR_GetFrameDataSet();
cv::Mat __FR_GetFrameWebCam(void);
static bool FRPriv_BufferNextDataSetFrame(void);
static bool FRPriv_ReadNextDataSetEntry(
        std::string& FramePath,
#if defined(CONFIG_STEREO)
        std::string& RightFramePath,
#endif
        fp64& TimeStamp);
static typeDecodedDataSetFrame FRPriv_DecodeDataSetFrame(
        const std::string& FramePath,
#if defined(CONFIG_STEREO)
        const std::string& RightFramePath,
#endif
        const fp64 TimeStamp);
static typePantoFrame FRPriv_FinalizeDataSetFrame( typeDecodedDataSetFrame Frame);
static void FRPriv_PreloadDataSetFrames(std::stop_token StopToken);

int FR_InitFrameGetter()
{
    if(PANTO_USE_DATASET == true)
    {
        if(reader.PreloadThread.joinable())
        {
            reader.PreloadThread.request_stop();
            reader.PreloadNotFull.notify_all();
            reader.PreloadThread.join();
        }

        const std::string DatasetPath =
            std::string(PANTO_DATASET_BASE_PATH) +
            std::string(panto_dataset_path);

#if defined(CONFIG_STEREO)
        reader.ImagePath = DatasetPath + "/cam0/data";
        reader.RightImagePath = DatasetPath + "/cam1/data";
#else
        reader.ImagePath = DatasetPath + "/" +
            std::string(panto_sequence_path);
#endif
        reader.OutputFrameIndex = 0;
        reader.BufferedFramePath.clear();
#if defined(CONFIG_STEREO)
        reader.BufferedRightFramePath.clear();
#endif
        reader.BufferedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
        reader.HasBufferedFrame = false;
        {
            std::lock_guard<std::mutex> Lock(reader.PreloadMutex);
            reader.PreloadedFrames.clear();
            reader.PreloadEndOfStream = false;
        }

        const std::string TimeStampPath =
            DatasetPath + "/cam0/data.csv";
#if defined(CONFIG_STEREO)
        const std::string RightTimeStampPath =
            DatasetPath + "/cam1/data.csv";
#endif

        if(reader.TimeStampFile.is_open())
        {
            reader.TimeStampFile.close();
        }

        reader.TimeStampFile.open(TimeStampPath);

        if(!reader.TimeStampFile.is_open())
        {
            LG_Log(LogSeverity::ERROR,
                    "[FR_InitFrameGetter] Failed to open timestamp file: %s\n",
                    TimeStampPath.c_str());
            return 1;
        }

#if defined(CONFIG_STEREO)
        if(reader.RightTimeStampFile.is_open())
        {
            reader.RightTimeStampFile.close();
        }

        reader.RightTimeStampFile.open(RightTimeStampPath);

        if(!reader.RightTimeStampFile.is_open())
        {
            LG_Log(LogSeverity::ERROR,
                    "[FR_InitFrameGetter] Failed to open right timestamp file: %s\n",
                    RightTimeStampPath.c_str());
            reader.TimeStampFile.close();
            return 1;
        }

        LG_Log(LogSeverity::DBG,
                "[FR_InitFrameGetter] Stereo dataset left: %s (%s), right: %s (%s)\n",
                reader.ImagePath.c_str(),
                TimeStampPath.c_str(),
                reader.RightImagePath.c_str(),
                RightTimeStampPath.c_str());
#else

        LG_Log(LogSeverity::DBG,
                "[FR_InitFrameGetter] Dataset images: %s, timestamps: %s\n",
                reader.ImagePath.c_str(),
                TimeStampPath.c_str());
#endif

        if(PANTO_DATASET_REALTIME_MODE)
        {
            reader.PreloadThread = std::jthread(
                    FRPriv_PreloadDataSetFrames);
        }
    }
    else
    {
#if defined(CONFIG_STEREO)
        LG_Log(LogSeverity::ERROR,
                "[FR_InitFrameGetter] CONFIG_STEREO requires a EuRoC-style dataset\n");
        return 1;
#else
        if(!cap.cap.open(0))
        {
            std::cerr << "Failed to open camera\n";
            return 1;
        }
        cap.isInit = true;
        cap.FrameIndex = 0;
#endif

    }
    return 0;
}

typePantoFrame FR_GetFrame(void)
{
    typePantoFrame PantoFrame{};
    if(PANTO_USE_DATASET == true)
    {
        typePantoFrame Frame = __FR_GetFrameDataSet();
        PantoFrame = Frame;
    }
    else
    {
#if defined(CONFIG_STEREO)
        LG_Log(LogSeverity::ERROR,
                "[FR_GetFrame] CONFIG_STEREO does not support webcam input\n");
        return
        {
            .Frame = cv::Mat{},
            .RightFrame = cv::Mat{},
            .TimeStamp = PANTO_TIMESTAMP_NOT_SET,
            .Path = "",
            .Descriptors{}
        };
#else
        cv::Mat Frame = __FR_GetFrameWebCam();
        const PantoClock::time_point CurrentTime = PantoClock::now();
        const fp64 TimeStamp = std::chrono::duration(CurrentTime - StartTime).count();
        PantoFrame.Frame = Frame; 
        PantoFrame.TimeStamp = TimeStamp;
#endif
    }
    return PantoFrame;
}

fp64 FR_PeekNextFrameTimeStamp(void)
{
    if(!PANTO_USE_DATASET)
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    if(PANTO_DATASET_REALTIME_MODE)
    {
        std::unique_lock<std::mutex> Lock(reader.PreloadMutex);
        reader.PreloadNotEmpty.wait(
                Lock,
                []()
                {
                    return !reader.PreloadedFrames.empty() ||
                        reader.PreloadEndOfStream;
                });
        return reader.PreloadedFrames.empty()
            ? PANTO_TIMESTAMP_NOT_SET
            : reader.PreloadedFrames.front().TimeStamp;
    }

    if(!FRPriv_BufferNextDataSetFrame())
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    return reader.BufferedTimeStamp;
}

fp64 FR_SkipNextFrame(void)
{
    if(!PANTO_USE_DATASET)
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    if(PANTO_DATASET_REALTIME_MODE)
    {
        std::unique_lock<std::mutex> Lock(reader.PreloadMutex);
        reader.PreloadNotEmpty.wait(
                Lock,
                []()
                {
                    return !reader.PreloadedFrames.empty() ||
                        reader.PreloadEndOfStream;
                });
        if(reader.PreloadedFrames.empty())
        {
            return PANTO_TIMESTAMP_NOT_SET;
        }

        const fp64 SkippedTimeStamp =
            reader.PreloadedFrames.front().TimeStamp;
        reader.PreloadedFrames.pop_front();
        Lock.unlock();
        reader.PreloadNotFull.notify_one();
        return SkippedTimeStamp;
    }

    if(!FRPriv_BufferNextDataSetFrame())
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    const fp64 SkippedTimeStamp = reader.BufferedTimeStamp;

    LG_Log(LogSeverity::DBG,
            "[FR_SkipNextFrame] Skipping frame %s at %.9f s\n",
            reader.BufferedFramePath.c_str(),
            SkippedTimeStamp);

    reader.BufferedFramePath.clear();
#if defined(CONFIG_STEREO)
    reader.BufferedRightFramePath.clear();
#endif
    reader.BufferedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
    reader.HasBufferedFrame = false;

    return SkippedTimeStamp;
}

cv::Mat __FR_GetFrameWebCam(void)
{
    if(!cap.isInit)
    {
        std::cerr << "Camera not initialized when calling FR_GetFrame\n";
    }

    cv::Mat frame;
    //get one frame
    if(!cap.cap.read(frame) || frame.empty())
    {
        std::cerr << "FR_GetFrame failed to get a frame\n";
        return {};
    }
    std::string path = "./colmap/images/frame" + std::to_string(cap.FrameIndex) + ".png";
    LG_Log(LogSeverity::DBG, "[FR_GetFrame] writing frame file %s\n", path.c_str());
    bool ret = cv::imwrite(path, frame);
    if(!ret)
    {
        perror("Failed to write image in getframe");
        return {};
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

typePantoFrame __FR_GetFrameDataSet()
{
    if(PANTO_DATASET_REALTIME_MODE)
    {
        std::unique_lock<std::mutex> Lock(reader.PreloadMutex);
        reader.PreloadNotEmpty.wait(
                Lock,
                []()
                {
                    return !reader.PreloadedFrames.empty() ||
                        reader.PreloadEndOfStream;
                });
        if(reader.PreloadedFrames.empty())
        {
            return
            {
                .Frame = cv::Mat{},
#if defined(CONFIG_STEREO)
                .RightFrame = cv::Mat{},
#endif
                .TimeStamp = PANTO_TIMESTAMP_NOT_SET,
                .Path = ""
            };
        }

        typeDecodedDataSetFrame Frame = std::move(reader.PreloadedFrames.front());
        reader.PreloadedFrames.pop_front();
        Lock.unlock();
        reader.PreloadNotFull.notify_one();
        return FRPriv_FinalizeDataSetFrame(std::move(Frame));
    }

    if(!FRPriv_BufferNextDataSetFrame())
    {
        LG_Log(LogSeverity::DBG,
                "[__FR_GetFrameDataSet] No next dataset frame available\n");
        return
        {
            .Frame = cv::Mat{},
#if defined(CONFIG_STEREO)
            .RightFrame = cv::Mat{},
#endif
            .TimeStamp = -1.0,
            .Path = ""
        };
    }

    const std::string FramePath = reader.BufferedFramePath;
#if defined(CONFIG_STEREO)
    const std::string RightFramePath = reader.BufferedRightFramePath;
#endif
    const fp64 TimeStamp = reader.BufferedTimeStamp;

    reader.BufferedFramePath.clear();
#if defined(CONFIG_STEREO)
    reader.BufferedRightFramePath.clear();
#endif
    reader.BufferedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
    reader.HasBufferedFrame = false;

    return FRPriv_FinalizeDataSetFrame(FRPriv_DecodeDataSetFrame(
            FramePath,
#if defined(CONFIG_STEREO)
            RightFramePath,
#endif
            TimeStamp));
}

static bool FRPriv_BufferNextDataSetFrame(void)
{
    if(reader.HasBufferedFrame)
    {
        return true;
    }

    if(!reader.TimeStampFile.is_open())
    {
        LG_Log(LogSeverity::ERROR,
                "[FRPriv_BufferNextDataSetFrame] Frame getter is not initialized\n");
        return false;
    }

    reader.HasBufferedFrame = FRPriv_ReadNextDataSetEntry(
            reader.BufferedFramePath,
#if defined(CONFIG_STEREO)
            reader.BufferedRightFramePath,
#endif
            reader.BufferedTimeStamp);
    return reader.HasBufferedFrame;
}

static bool FRPriv_ReadDataSetEntry(
        std::ifstream& TimeStampFile,
        const std::string& ImagePath,
        std::string& FramePath,
        u64& TimeStampNanoseconds)
{
    std::string Line;

    while(std::getline(TimeStampFile, Line))
    {
        if(Line.empty() || Line[0] == '#')
        {
            continue;
        }

        if(!Line.empty() && Line.back() == '\r')
        {
            Line.pop_back();
        }

        std::stringstream Stream(Line);
        std::string TimeStampToken;
        std::string FileName;

        if(!std::getline(Stream, TimeStampToken, ',') ||
           !std::getline(Stream, FileName))
        {
            continue;
        }

        TimeStampNanoseconds = std::stoull(TimeStampToken);
        FramePath = ImagePath + "/" + FileName;
        return true;
    }

    return false;
}

static bool FRPriv_ReadNextDataSetEntry(
        std::string& FramePath,
#if defined(CONFIG_STEREO)
        std::string& RightFramePath,
#endif
        fp64& TimeStamp)
{
    u64 TimeStampNanoseconds{};
    const bool HasLeftFrame = FRPriv_ReadDataSetEntry(
            reader.TimeStampFile,
            reader.ImagePath,
            FramePath,
            TimeStampNanoseconds);

#if defined(CONFIG_STEREO)
    u64 RightTimeStampNanoseconds{};
    const bool HasRightFrame = FRPriv_ReadDataSetEntry(
            reader.RightTimeStampFile,
            reader.RightImagePath,
            RightFramePath,
            RightTimeStampNanoseconds);

    if(HasLeftFrame != HasRightFrame)
    {
        LG_Log(LogSeverity::ERROR,
                "[FRPriv_ReadNextDataSetEntry] cam0 and cam1 have different frame counts\n");
        return false;
    }

    if(!HasLeftFrame)
    {
        return false;
    }

    if(TimeStampNanoseconds != RightTimeStampNanoseconds)
    {
        LG_Log(LogSeverity::ERROR,
                "[FRPriv_ReadNextDataSetEntry] Stereo timestamp mismatch: cam0 = %llu, cam1 = %llu\n",
                static_cast<unsigned long long>(TimeStampNanoseconds),
                static_cast<unsigned long long>(RightTimeStampNanoseconds));
        return false;
    }
#else
    if(!HasLeftFrame)
    {
        return false;
    }
#endif

    TimeStamp = static_cast<fp64>(TimeStampNanoseconds) * 1e-9;
    return true;
}

static typeDecodedDataSetFrame FRPriv_DecodeDataSetFrame(
        const std::string& FramePath,
#if defined(CONFIG_STEREO)
        const std::string& RightFramePath,
#endif
        const fp64 TimeStamp)
{
    typeDecodedDataSetFrame Result{};
    Result.TimeStamp = TimeStamp;
    Result.SourcePath = FramePath;
    Result.Gray = cv::imread(FramePath, cv::IMREAD_GRAYSCALE);
#if defined(CONFIG_STEREO)
    Result.RightGray = cv::imread(RightFramePath, cv::IMREAD_GRAYSCALE);
#endif
    return Result;
}

static typePantoFrame FRPriv_FinalizeDataSetFrame(
        typeDecodedDataSetFrame Frame)
{
    if(Frame.Gray.empty()
#if defined(CONFIG_STEREO)
            || Frame.RightGray.empty()
#endif
      )
    {
        return
        {
            .Frame = cv::Mat{},
#if defined(CONFIG_STEREO)
            .RightFrame = cv::Mat{},
#endif
            .TimeStamp = PANTO_TIMESTAMP_NOT_SET,
            .Path = "",
            .Descriptors{}
                
        };
    }

    const std::filesystem::path SourcePath(Frame.SourcePath);
    const std::filesystem::path WritePath =
        std::filesystem::path("./colmap/images") / ("frame" + std::to_string(Frame.TimeStamp) + SourcePath.extension().string());

    std::error_code FileError;
    std::filesystem::create_hard_link(SourcePath, WritePath, FileError);

    if(FileError)
    {
        FileError.clear();
        std::filesystem::copy_file(
                SourcePath,
                WritePath,
                std::filesystem::copy_options::overwrite_existing,
                FileError);
    }

    if(FileError)
    {
        LG_Log(LogSeverity::ERROR,
                "[FRPriv_FinalizeDataSetFrame] Failed to link or copy %s to %s: %s\n",
                SourcePath.c_str(), WritePath.c_str(),
                FileError.message().c_str());
        return
        {
            .Frame = cv::Mat{},
#if defined(CONFIG_STEREO)
            .RightFrame = cv::Mat{},
#endif
            .TimeStamp = PANTO_TIMESTAMP_NOT_SET,
            .Path = "",
            .Descriptors{}
        };
    }

    return
    {
        .Frame = std::move(Frame.Gray),
#if defined(CONFIG_STEREO)
        .RightFrame = std::move(Frame.RightGray),
#endif
        .TimeStamp = Frame.TimeStamp,
        .Path = WritePath.string(),
        .Descriptors = std::move(Frame.Descriptors)
    };
}

static void FRPriv_PreloadDataSetFrames(std::stop_token StopToken)
{
    while(!StopToken.stop_requested())
    {
        std::string FramePath;
#if defined(CONFIG_STEREO)
        std::string RightFramePath;
#endif
        fp64 TimeStamp = PANTO_TIMESTAMP_NOT_SET;
        if(!FRPriv_ReadNextDataSetEntry(
                    FramePath,
#if defined(CONFIG_STEREO)
                    RightFramePath,
#endif
                    TimeStamp))
        {
            std::lock_guard<std::mutex> Lock(reader.PreloadMutex);
            reader.PreloadEndOfStream = true;
            reader.PreloadNotEmpty.notify_all();
            return;
        }

        typeDecodedDataSetFrame Frame = FRPriv_DecodeDataSetFrame(
                FramePath,
#if defined(CONFIG_STEREO)
                RightFramePath,
#endif
                TimeStamp);

        if(!Frame.Gray.empty()
#if defined(CONFIG_STEREO)
                && !Frame.RightGray.empty()
#endif
          )
        {
            Frame.Descriptors = EP_GetDescriptors(Frame.Gray);
        }

        std::unique_lock<std::mutex> Lock(reader.PreloadMutex);
        if(!reader.PreloadNotFull.wait(
                    Lock,
                    StopToken,
                    []()
                    {
                        return reader.PreloadedFrames.size() <
                            PANTO_REALTIME_FRAME_QUEUE_CAPACITY;
                    }))
        {
            return;
        }
        reader.PreloadedFrames.push_back(std::move(Frame));
        Lock.unlock();
        reader.PreloadNotEmpty.notify_one();
    }
}

#include "../include/FR_Frames.hpp"
#include <chrono>
#include <fstream>
#include <sstream>

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
    struct dataset_read 
    {
        std::string ImagePath;
        std::ifstream TimeStampFile;
        u64 OutputFrameIndex;
        std::string BufferedFramePath;
        fp64 BufferedTimeStamp;
        bool HasBufferedFrame;
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

int FR_InitFrameGetter()
{
    if(PANTO_USE_DATASET == true)
    {
        const std::string DatasetPath =
            std::string(PANTO_DATASET_BASE_PATH) +
            std::string(panto_dataset_path);

        reader.ImagePath = DatasetPath + "/" +
            std::string(panto_sequence_path);
        reader.OutputFrameIndex = 0;
        reader.BufferedFramePath.clear();
        reader.BufferedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
        reader.HasBufferedFrame = false;

        const std::string TimeStampPath =
            DatasetPath + "/cam0/data.csv";

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

        LG_Log(LogSeverity::DBG,
                "[FR_InitFrameGetter] Dataset images: %s, timestamps: %s\n",
                reader.ImagePath.c_str(),
                TimeStampPath.c_str());
    }
    else
    {
        if(!cap.cap.open(0))
        {
            std::cerr << "Failed to open camera\n";
            return 1;
        }
        cap.isInit = true;
        cap.FrameIndex = 0;

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
        cv::Mat Frame = __FR_GetFrameWebCam();
        const PantoClock::time_point CurrentTime = PantoClock::now();
        const fp64 TimeStamp = std::chrono::duration(CurrentTime - StartTime).count();
        PantoFrame.Frame = Frame; 
        PantoFrame.TimeStamp = TimeStamp;
    }
    return PantoFrame;
}

fp64 FR_PeekNextFrameTimeStamp(void)
{
    if(!PANTO_USE_DATASET)
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    if(!FRPriv_BufferNextDataSetFrame())
    {
        return PANTO_TIMESTAMP_NOT_SET;
    }

    return reader.BufferedTimeStamp;
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
    if(!FRPriv_BufferNextDataSetFrame())
    {
        LG_Log(LogSeverity::DBG,
                "[__FR_GetFrameDataSet] No next dataset frame available\n");
        return
        {
            .Frame = cv::Mat{},
            .TimeStamp = -1.0,
            .Path = ""
        };
    }

    const std::string FramePath = reader.BufferedFramePath;
    const fp64 TimeStamp = reader.BufferedTimeStamp;

    reader.BufferedFramePath.clear();
    reader.BufferedTimeStamp = PANTO_TIMESTAMP_NOT_SET;
    reader.HasBufferedFrame = false;

    LG_Log(LogSeverity::DBG, "[__FR_GetFrameDataSet] Getting frame %s\n", FramePath.c_str());
    cv::Mat Frame = cv::imread(FramePath, cv::IMREAD_COLOR);

    if(Frame.empty())
    {
        LG_Log(LogSeverity::DBG,
                "[__FR_GetFrameDataSet] cv::imread returned empty frame for %s\n",
                FramePath.c_str());
        return 
        {
            .Frame = cv::Mat{},
            .TimeStamp = -1.0,
            .Path = "" 
        };
    }

    const std::string WritePath =
        "./colmap/images/frame" +
        std::to_string(reader.OutputFrameIndex) +
        ".png";

    cv::imwrite(WritePath, Frame);

    cv::Mat Gray;
    cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);

    reader.OutputFrameIndex++;

    return 
    {
        .Frame = Gray,
        .TimeStamp = TimeStamp,
        .Path = WritePath
    };
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

    std::string Line;

    while(std::getline(reader.TimeStampFile, Line))
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

        reader.BufferedTimeStamp =
            static_cast<fp64>(std::stoull(TimeStampToken)) * 1e-9;
        reader.BufferedFramePath = reader.ImagePath + "/" + FileName;
        reader.HasBufferedFrame = true;
        return true;
    }

    return false;
}

#include "../include/LG_Logging.hpp"
#include <cstdio>

struct Logger glogger;
bool gloggerisinit = false;
void LGPriv_Log(FILE* fp, const char*fmt, ...);

static std::string LG_MakeTimestampedLogPath(const std::string& basePath, const std::string& Type)
{
    namespace fs = std::filesystem;

    fs::path path(basePath);
    fs::path HistoricalPath = path.parent_path() / "Historical";

    if(fs::exists(path) && fs::is_directory(path))
    {
        fs::create_directories(HistoricalPath);

        for(const fs::directory_entry& entry : fs::directory_iterator(path))
        {
            const fs::path destination = HistoricalPath / entry.path().filename();

            fs::rename(entry.path(), destination);
        }
    }

    path = path / fs::path(Type + "PantoLOG");

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};

#if defined(_WIN32)
    localtime_s(&tm, &now_time);
#else
    localtime_r(&now_time, &tm);
#endif

    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

    fs::path dir = path.parent_path();
    std::string stem = path.stem().string();
    std::string ext = path.extension().string();

    fs::path newPath = dir / (stem + "_" + timestamp.str() + ext);

    return newPath.string();
}

void LG_InitLogger()
{
    if (gloggerisinit) return;

    std::string logPath = PANTO_LOGPATH;

    const std::string LogPathDebug = LG_MakeTimestampedLogPath(logPath, "DEBUG");
    const std::string LogPathData = LG_MakeTimestampedLogPath(logPath, "DATA");
    const std::string LogPathError = LG_MakeTimestampedLogPath(logPath, "ERROR");

    glogger =
    {
        .DebugPath = LogPathDebug,
        .DataPath = LogPathData,
        .ErrorPath = LogPathError,
        .Debugfp = std::fopen(LogPathDebug.c_str(), "a"),
        .Datafp = std::fopen(LogPathData.c_str(), "a"),
        .Errorfp = std::fopen(LogPathError.c_str(), "a")
    };

    if (!glogger.Debugfp || !glogger.Datafp || !glogger.Errorfp) 
    {
        printf("Failed logging init\n");
        fflush(stdout);
        exit(1);
    }

    gloggerisinit = true;

    LG_Log(LogSeverity::DBG, "Logger initiated\n");
    LG_Log(LogSeverity::DBG, "Log file: %s\n", logPath.c_str());
}

void LG_CloseLogger()
{
    if (glogger.Debugfp)
    {
        std::fclose(glogger.Debugfp);
        glogger.Debugfp = nullptr;
    }
    if (glogger.Datafp)
    {
        std::fclose(glogger.Datafp);
        glogger.Datafp = nullptr;
    }
    if (glogger.Errorfp)
    {
        std::fclose(glogger.Errorfp);
        glogger.Errorfp = nullptr;
    }

    gloggerisinit = false;
}

void LG_Log(LogSeverity severity, const char* fmt, ...)
{
    if (!gloggerisinit)
    {
        LG_InitLogger();
    }

    FILE* fp = nullptr;

    switch (severity)
    {
        case LogSeverity::DBG:
            fp = glogger.Debugfp;
            break;

        case LogSeverity::DATA:
            fp = glogger.Datafp;
            break;

        case LogSeverity::ERROR:
            fp = glogger.Errorfp;
            break;
    }

    if (!fp)
    {
        std::fprintf(stderr, "Logger FILE pointer is null\n");
        return;
    }

    va_list args;
    va_start(args, fmt);

    va_list stdoutArgs;
    va_copy(stdoutArgs, args);

    std::vfprintf(fp, fmt, args);

    if (CONFIG_PRINT_LOGS_TO_STDOUT)
    {
        std::vprintf(fmt, stdoutArgs);
    }

    va_end(stdoutArgs);
    va_end(args);

    std::fflush(fp);
}


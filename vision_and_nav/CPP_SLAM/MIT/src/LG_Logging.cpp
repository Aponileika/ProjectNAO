#include "../include/LG_Logging.hpp"

struct Logger glogger;
bool gloggerisinit = false;

static std::string LG_MakeTimestampedLogPath(const std::string& basePath)
{
    namespace fs = std::filesystem;

    fs::path path(basePath);

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

    std::string logPath = LOGPATH;

    if (std::filesystem::exists(logPath))
    {
        logPath = LG_MakeTimestampedLogPath(logPath);
    }

    glogger =
    {
        logPath,
        std::fopen(logPath.c_str(), "a")
    };

    if (!glogger.fp) exit(1);

    gloggerisinit = true;

    LG_Log("Logger initiated\n");
    LG_Log("Log file: %s\n", logPath.c_str());
}

void LG_CloseLogger()
{
    if (glogger.fp)
    {
        std::fclose(glogger.fp);
        glogger.fp = nullptr;
    }

    gloggerisinit = false;
}

void LG_Log(const char* fmt, ...)
{
    if (!gloggerisinit) LG_InitLogger();

    va_list args;
    va_start(args, fmt);

    va_list args_copy;
    va_copy(args_copy, args);

    std::vfprintf(glogger.fp, fmt, args);
    std::vprintf(fmt, args_copy);

    va_end(args_copy);
    va_end(args);

    std::fflush(glogger.fp);
}

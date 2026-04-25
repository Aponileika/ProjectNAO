#include "../include/LG_Logging.hpp"

struct Logger glogger;
bool gloggerisinit = false;

void LG_InitLogger()
{
    if(gloggerisinit)return;
    glogger = 
    {
        LOGPATH,
        std::fopen(LOGPATH, "a")
    };
    if(!glogger.fp)exit(1);
    gloggerisinit = true;
    LG_Log("Logger initiated\n");
}

void LG_CloseLogger()
{
    fclose(glogger.fp);
}

void LG_Log(const char* fmt, ...)
{
    if(!gloggerisinit)LG_InitLogger();

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



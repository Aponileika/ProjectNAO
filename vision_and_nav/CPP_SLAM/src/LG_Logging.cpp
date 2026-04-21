#include "../include/LG_Logging.hpp"

struct Logger glogger;
bool gloggerisinit = false;

void LG_InitLogger()
{
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
    assert(gloggerisinit == true);

    va_list args;
    va_start(args, fmt);
    std::vfprintf(glogger.fp, fmt,  args);
    va_end(args);
    va_list stdargs;
    va_start(stdargs, fmt);
    vprintf(fmt, args);
    va_end(stdargs);

}



#ifndef __LG_LOGGING_HPP_
#define __LG_LOGGING_HPP_
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include "Config.hpp"
#include "CArenaAlloc.h"

struct Logger
{
    std::string path;
    FILE* fp;
};

enum class LogSeverity : u8
{
    DBG = 0, // Everything is fine
    ERROR = 1, // Must reset SLAM
    DATA = 2 // Must reset SLAM
};


void LG_InitLogger();
void LG_CloseLogger();
void LG_Log(LogSeverity severity, const char* fmt, ...);

#endif //__LG_LOGGING_HPP_

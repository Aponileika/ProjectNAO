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

#define LOGPATH "/Users/Jonathan/Programmering/FIA/ProjectNAO/vision_and_nav/CPP_SLAM/logs/log.txt"

struct Logger
{
    std::string path;
    FILE* fp;
};

extern struct Logger glogger;

void LG_InitLogger();
void LG_CloseLogger();
void LG_Log(const char* fmt, ...);

#endif //__LG_LOGGING_HPP_

/**
 * File: tprintf.cpp
 * Author: Jan Feitsma
 * Creation: January 2019
 *
 */

#include "tprintf.hpp"

#include <cstdio>
#include <ctime>
#include <sys/time.h>
#include <string>

std::string get_timestamp_now()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    struct tm tm;
    localtime_r(&tv.tv_sec, &tm);
    char timebuf[80] = {0};
    snprintf(timebuf, sizeof(timebuf), "%04d-%02d-%02d,%02d:%02d:%02d.%06d",
             1900 + tm.tm_year, 1 + tm.tm_mon, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec, (int)tv.tv_usec);
    return std::string(timebuf);
}

void _tprintf_start()
{
    std::string timestr = get_timestamp_now();
    printf("%s ", timestr.c_str());
}

void _tprintf_end()
{
    printf("\n");
    fflush(stdout);
}

void _tprintf_wrap(char const *msg)
{
    _tprintf_start();
    printf("%s", msg);
    _tprintf_end();
}

void _tprintf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    char *tmp = 0;
    int res = vasprintf(&tmp, fmt, ap);
    va_end(ap);

    if (res != -1)
    {
        _tprintf_wrap(tmp);
        free(tmp);
    } // else: vasprintf strangely enough failed; no cleanup nor reporting needed
}

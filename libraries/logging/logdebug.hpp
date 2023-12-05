#ifndef _MRA_LIBRARIES_LOGGING_LOGDEBUG_HPP
#define _MRA_LIBRARIES_LOGGING_LOGDEBUG_HPP


// uncomment to enable internal debug logging
// this is intended only for developing libraries/logging

//#define LOGDEBUG_ENABLED

#ifdef LOGDEBUG_ENABLED
#include <cstdio>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

inline std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::microseconds(microseconds)).count();

    auto fractional_seconds = microseconds - (seconds * 1000000);

    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::gmtime(&time);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setw(6) << std::setfill('0') << fractional_seconds;

    return oss.str();
}

// Define the LOGDEBUG macro
#define LOGDEBUG(fmt, ...) \
    do { \
        if constexpr (true) { \
            std::printf("%s %s(%d) - ", getCurrentTimestamp().c_str(), __FILE__, __LINE__); \
            std::printf(fmt, ##__VA_ARGS__); \
            std::printf("\n"); \
            std::fflush(stdout); \
        } \
    } while (false)

#else // LOGDEBUG_ENABLED is not defined, define LOGDEBUG as empty
#define LOGDEBUG(fmt, ...) do {} while (false)
#endif // LOGDEBUG_ENABLED


#endif // #ifndef _MRA_LIBRARIES_LOGGING_LOGDEBUG_HPP


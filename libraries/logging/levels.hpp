#ifndef _MRA_LIBRARIES_LOGGING_LEVELS_HPP
#define _MRA_LIBRARIES_LOGGING_LEVELS_HPP

// workaround for TRACE macro that may have been defined already
#ifdef TRACE
#undef TRACE
#endif


namespace MRA::Logging
{

enum LogLevel
{
    CRITICAL,
    ERROR,
    WARNING,
    INFO,
    DEBUG,
    TRACE
};

}

#endif // #ifndef _MRA_LIBRARIES_LOGGING_LEVELS_HPP


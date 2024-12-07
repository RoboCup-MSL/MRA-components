#ifndef _MRA_LIBRARIES_LOGGING_DUMMIES_HPP
#define _MRA_LIBRARIES_LOGGING_DUMMIES_HPP


// use these macros to disable logging compile-time
// the real definitions are in libraries/logging/macros.hpp

#define MRA_LOG_TICK() do {} while (0)
#define MRA_LOG_CRITICAL(...) do {} while (0)
#define MRA_LOG_ERROR(...) do {} while (0)
#define MRA_LOG_WARNING(...) do {} while (0)
#define MRA_LOG_INFO(...) do {} while (0)
#define MRA_LOG_DEBUG(...) do {} while (0)
#define MRA_TRACE_FUNCTION() do {} while (0)
#define MRA_TRACE_TEST_FUNCTION() do {} while (0)
#define MRA_TRACE_FUNCTION_INPUTS(...) do {} while (0)
#define MRA_TRACE_FUNCTION_OUTPUTS(...) do {} while (0)
#define MRA_TRACE_FUNCTION_INPUT(varname) do {} while (0)
#define MRA_TRACE_FUNCTION_OUTPUT(varname) do {} while (0)

#endif // #ifndef _MRA_LIBRARIES_LOGGING_DUMMIES_HPP


#ifndef MRA_TRACING_HPP
#define MRA_TRACING_HPP

// inspired by MRA macros

#include "mra_tracing/macromap.h"
#include "mra_tracing/backend.hpp"

// trace function call and duration, without arguments
#define TRACE_FUNCTION() MRA::tracing::FunctionRecord scoped(\
    MRA::tracing::SourceLoc(__FILE__, __LINE__, __FUNCTION__) ); \
    scoped.flush_input()

// multi-argument function i/o logging
#define TRACE_FUNCTION_INPUT_PAIR(v) scoped.add_input(#v, v);
#define TRACE_FUNCTION_INPUTS(...) \
    MRA::tracing::FunctionRecord scoped( \
        MRA::tracing::SourceLoc(__FILE__, __LINE__, __FUNCTION__) \
    ); \
    MAP(TRACE_FUNCTION_INPUT_PAIR, __VA_ARGS__) \
    scoped.flush_input()

#define TRACE_FUNCTION_OUTPUT_PAIR(v) scoped.add_output(#v, v);
#define TRACE_FUNCTION_OUTPUTS(...) \
    MAP(TRACE_FUNCTION_OUTPUT_PAIR, __VA_ARGS__) \

#endif // MRA_TRACING_HPP

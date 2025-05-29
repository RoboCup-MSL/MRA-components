#ifndef TRACING_HPP
#define TRACING_HPP

// inspired by MRA macros

#include "tracing/macromap.h"
#include "tracing/backend.hpp"

// trace function call and duration, without arguments
#define TRACE_FUNCTION() tracing::FunctionRecord scoped(\
    tracing::SourceLoc(__FILE__, __LINE__, __FUNCTION__) ); \
    scoped.flush_input()

// multi-argument function i/o logging
#define TRACE_FUNCTION_INPUT_PAIR(v) scoped.add_input(#v, v);
#define TRACE_FUNCTION_INPUTS(...) \
    tracing::FunctionRecord scoped( \
        tracing::SourceLoc(__FILE__, __LINE__, __FUNCTION__) \
    ); \
    MAP(TRACE_FUNCTION_INPUT_PAIR, __VA_ARGS__) \
    scoped.flush_input()

#define TRACE_FUNCTION_OUTPUT_PAIR(v) scoped.add_output(#v, v);
#define TRACE_FUNCTION_OUTPUTS(...) \
    MAP(TRACE_FUNCTION_OUTPUT_PAIR, __VA_ARGS__) \

#endif // TRACING_HPP

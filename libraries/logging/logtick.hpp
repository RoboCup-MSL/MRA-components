#ifndef _MRA_LIBRARIES_LOGGING_LOGTICK_HPP
#define _MRA_LIBRARIES_LOGGING_LOGTICK_HPP

#include "abstract_interface.hpp"
#include "backend.hpp"
#include "json_convert.hpp"
#include "logdebug.hpp"
#include "control.hpp"
#include <sstream>


namespace MRA::Logging
{

typedef google::protobuf::Timestamp Tt;

// template for use in scoped tick logging
// see also logger ideas and requirements in https://github.com/janfeitsma/MRA-prototype/issues/10
// TODO: there can be large (binary) data present, which probably should be automatically filtered, at least when dumping to stdout
template <typename Ti, typename Tp, typename Ts, typename To, typename Td>
class LogTick
{
public:
    LogTick(std::string const &componentName, std::string const &componentRelPath, std::string const &fileName, int lineNumber, Tt const &timestamp, Ti const &input, Tp const &params, Ts *state, To *output, Td *diagnostics, int *error_value)
    :
        // store data for inspection later
        _componentName(componentName),
        _componentRelPath(componentRelPath),
        _fileName(fileName),
        _lineNumber(lineNumber),
        _t(timestamp),
        _t0(google::protobuf::util::TimeUtil::GetCurrentTime()),
        _input(input),
        _params(params),
        _state(state),
        _output(output),
        _diagnostics(diagnostics),
        _err(error_value)
    {
        start();
    }

    ~LogTick()
    {
        end();
    }

    void start()
    {
        // get configuration to use for this tick (do not allow logging only start or only end of tick)
        LOGDEBUG("LogTick.start componentName %s", _componentName.c_str());
        _cfg = control::getConfiguration(_componentName);
        LOGDEBUG("LogTick.start config %s", MRA::convert_proto_to_json_str(_cfg).c_str());
        // dispatch to backend
        if (_cfg.enabled())
        {
            backend::reconfigure(_cfg);
            // if so configured, dump data for binary file
            backend::logTickStart(_componentName, _fileName, _lineNumber, _cfg, _bindata, _counter, _t, _input, _params, *_state);
        }
    }

    void end()
    {
        // dispatch to backend
        if (_cfg.enabled())
        {
            // calculate tick duration
            auto elapsed = google::protobuf::util::TimeUtil::GetCurrentTime() - _t0;
            double duration_sec = 1e-9 * google::protobuf::util::TimeUtil::DurationToNanoseconds(elapsed);
            // call backend
            backend::logTickEnd(_componentName, _componentRelPath, _fileName, _lineNumber, _cfg, _bindata, _counter, _t0, duration_sec, *_err, *_state, *_output, *_diagnostics);
        }
        // update counter for next tick
        _counter++;
    }

private:
    // store data for logging at destruction (when tick ends, the logged object goes out of scope)
    Tt          _t0;
    Tt          _t;
    Ti const   &_input;
    Tp const   &_params;
    Ts         *_state;
    To         *_output;
    Td         *_diagnostics;
    int        *_err;
    static int  _counter;
    std::string _componentName;
    std::string _componentRelPath;
    std::string _fileName;
    int         _lineNumber;
    MRA::Datatypes::LogSpec _cfg;
    std::ostringstream _bindata;

}; // template class LogTick

// initialize the static counter
template <typename Ti, typename Tp, typename Ts, typename To, typename Tl>
int LogTick<Ti, Tp, Ts, To, Tl>::_counter = 0;

} // namespace MRA::Logging

#endif // #ifndef _MRA_LIBRARIES_LOGGING_LOGTICK_HPP



#include "tracing/tracing.hpp"
#include <unistd.h>
#include <mutex>
#include <filesystem>

#define MAX_LINE_LENGTH 4096

tracing::FunctionRecord::FunctionRecord(tracing::SourceLoc loc)
    : loc_(loc)
    , timestamp_start_(std::chrono::high_resolution_clock::now())
{
}

tracing::FunctionRecord::~FunctionRecord()
{
    flush_output(std::chrono::high_resolution_clock::now());
}

void tracing::FunctionRecord::add_input(std::string const &varname, int value)
{
    input_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_input(std::string const &varname, float value)
{
    input_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_input(std::string const &varname, double value)
{
    input_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_input(std::string const &varname, bool value)
{
    input_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_input(std::string const &varname, std::string const &value)
{
    input_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_output(std::string const &varname, int value)
{
    output_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_output(std::string const &varname, float value)
{
    output_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_output(std::string const &varname, double value)
{
    output_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_output(std::string const &varname, bool value)
{
    output_data_.emplace_back(varname, value);
}

void tracing::FunctionRecord::add_output(std::string const &varname, std::string const &value)
{
    output_data_.emplace_back(varname, value);
}

std::string tracing::FunctionRecord::convert_to_json(std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> const &data)
{
    std::string js = "{";
    bool first = true;
    for (const auto& item : data) {
        if (!first) js += ",";
        first = false;
        js += "\"" + item.first + "\":";
        if (std::holds_alternative<int>(item.second)) {
            js += std::to_string(std::get<int>(item.second));
        } else if (std::holds_alternative<double>(item.second)) {
            js += std::to_string(std::get<double>(item.second));
        } else if (std::holds_alternative<bool>(item.second)) {
            js += (std::get<bool>(item.second) ? "true" : "false");
        } else if (std::holds_alternative<std::string>(item.second)) {
            js += "\"" + std::get<std::string>(item.second) + "\"";
        }
    }
    js += "}";
    return js;
}

std::string sanitize(std::string const &s)
{
    std::string result = s;
    std::string search = "\n", replace = "\\n";
    size_t pos;
    while ((pos = result.find(search)) != std::string::npos) {
        result.replace(pos, 1, replace);
    }
    return result;
}

void append_to_tracefile(std::string const &filename, std::string const &line)
{
    // no logging if filename is empty
    if (filename.empty()) {
        return;
    }
    // set a mutex
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    // append to file
    FILE *fp = fopen(filename.c_str(), "a");
    if (fp == nullptr) {
        throw std::runtime_error("cannot open trace file " + filename);
    }
    if (fputs(line.c_str(), fp) == EOF) {
        fclose(fp);
        throw std::runtime_error("cannot write to trace file " + filename);
    }
    if (fputs("\n", fp) == EOF) {
        fclose(fp);
        throw std::runtime_error("cannot write to trace file " + filename);
    }
    // flush
    if (fflush(fp) != 0) {
        fclose(fp);
        throw std::runtime_error("cannot flush trace file " + filename);
    }
    if (fclose(fp) != 0) {
        throw std::runtime_error("cannot close trace file " + filename);
    }
}

// TODO: static variable(s) to speedup?

std::string determine_trace_filename(std::string const &loc_filename)
{
    // we cannot write to /tmp because of docker container permissions
    // reuse log folder as produced by ros2
    // it is assumed that the launch script creates the folder and symlink
    (void)loc_filename; // prevent unused variable warning
    std::string log_folder = "/workspace/log/latest";
    // check if the folder exists - if not, problem in launch script
    if (!std::filesystem::exists(log_folder)) {
        return "";
    }
    pid_t pid = getpid(); // get pid
    std::string result = log_folder + "/process_" + std::to_string(pid) + ".trace";
    return result;
}

std::string format_time_point(tracing::timestamp_t const &timestamp, const char *format)
{
    // convert to time_t
    auto time = std::chrono::high_resolution_clock::to_time_t(timestamp);
    // convert to tm
    std::tm tm = *std::localtime(&time);
    // format the time
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), format, &tm);
    return std::string(buffer);
}

void dispatch_trace_line(tracing::timestamp_t const &timestamp, tracing::SourceLoc const &loc, std::string const &details)
{
    // sanitize string, e.g. replace newlines with '\n'
    std::string s = sanitize(details);
    // prevent overflow
    if (s.length() > MAX_LINE_LENGTH) {
        s = s.substr(0, MAX_LINE_LENGTH-3) + "...";
    }
    // prepend timestamp and source location
    std::string timestamp_str = format_time_point(timestamp, "%Y-%m-%dT%H:%M:%S");
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch());
    auto microseconds = duration.count() % 1000000;
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(6) << microseconds;
    timestamp_str += "." + oss.str();
    std::string loc_str = std::string(loc.filename) + ":" + std::to_string(loc.line) + "," + loc.funcname;
    std::string line = timestamp_str + " " + loc_str + " " + s;
    // append to file
    append_to_tracefile(determine_trace_filename(loc.filename), line);
}

void tracing::FunctionRecord::flush_input()
{
    std::string js = convert_to_json(input_data_);
    dispatch_trace_line(timestamp_start_, loc_, std::string("> ") + js.c_str());
}

void tracing::FunctionRecord::flush_output(tracing::timestamp_t const &timestamp)
{
    std::string js = convert_to_json(output_data_);
    dispatch_trace_line(timestamp, loc_, std::string("< ") + js.c_str());
}

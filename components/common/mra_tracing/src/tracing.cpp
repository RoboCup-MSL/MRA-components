#include "mra_tracing/tracing.hpp"
#include <unistd.h>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <cstdlib>

#define MAX_LINE_LENGTH 4096

using namespace MRA::tracing;

// Global variables for trace metadata
static std::atomic<bool> g_header_written{false};

// Forward declaration
void write_trace_header();

FunctionRecord::FunctionRecord(SourceLoc loc)
    : loc_(loc)
    , timestamp_start_(std::chrono::high_resolution_clock::now())
{
}

FunctionRecord::~FunctionRecord()
{
    flush_output(std::chrono::high_resolution_clock::now());
}

void FunctionRecord::add_input(std::string const &varname, int value)
{
    input_data_.emplace_back(varname, value);
}

void FunctionRecord::add_input(std::string const &varname, float value)
{
    input_data_.emplace_back(varname, value);
}

void FunctionRecord::add_input(std::string const &varname, double value)
{
    input_data_.emplace_back(varname, value);
}

void FunctionRecord::add_input(std::string const &varname, bool value)
{
    input_data_.emplace_back(varname, value);
}

void FunctionRecord::add_input(std::string const &varname, std::string const &value)
{
    input_data_.emplace_back(varname, value);
}

void FunctionRecord::add_output(std::string const &varname, int value)
{
    output_data_.emplace_back(varname, value);
}

void FunctionRecord::add_output(std::string const &varname, float value)
{
    output_data_.emplace_back(varname, value);
}

void FunctionRecord::add_output(std::string const &varname, double value)
{
    output_data_.emplace_back(varname, value);
}

void FunctionRecord::add_output(std::string const &varname, bool value)
{
    output_data_.emplace_back(varname, value);
}

void FunctionRecord::add_output(std::string const &varname, std::string const &value)
{
    output_data_.emplace_back(varname, value);
}

std::string FunctionRecord::convert_to_json(std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> const &data)
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

// Static variables for optimization
static std::string g_trace_dir = "";
static bool g_trace_dir_initialized = false;
static bool g_tracing_enabled = false;

std::string determine_trace_filename(std::string const &loc_filename)
{
    (void)loc_filename; // prevent unused variable warning

    // Initialize trace directory once
    if (!g_trace_dir_initialized) {
        g_trace_dir_initialized = true;

        // Try MRA_TRACING_DIR environment variable first
        const char* env_trace_dir = std::getenv("MRA_TRACING_DIR");
        if (env_trace_dir != nullptr) {
            g_trace_dir = std::string(env_trace_dir);
        } else {
            // Fallback to /tmp/mra_tracing
            g_trace_dir = "/tmp/mra_tracing";
        }

        // Check if the directory exists - if not, disable tracing
        g_tracing_enabled = std::filesystem::exists(g_trace_dir) && std::filesystem::is_directory(g_trace_dir);
    }

    // Return empty string if tracing is disabled (optimization for no-op)
    if (!g_tracing_enabled) {
        return "";
    }

    pid_t pid = getpid();
    std::string result = g_trace_dir + "/process_" + std::to_string(pid) + ".trace";
    return result;
}

std::string format_time_point(timestamp_t const &timestamp, const char *format)
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

void dispatch_trace_line(timestamp_t const &timestamp, SourceLoc const &loc, std::string const &details)
{
    // Early return if tracing is disabled (optimization for no-op)
    std::string trace_filename = determine_trace_filename(loc.filename);
    if (trace_filename.empty()) {
        return;
    }

    // Write header if this is the first trace line
    write_trace_header();
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
    append_to_tracefile(trace_filename, line);
}

void FunctionRecord::flush_input()
{
    std::string js = convert_to_json(input_data_);
    dispatch_trace_line(timestamp_start_, loc_, std::string("> ") + js.c_str());
}

void FunctionRecord::flush_output(timestamp_t const &timestamp)
{
    std::string js = convert_to_json(output_data_);
    dispatch_trace_line(timestamp, loc_, std::string("< ") + js.c_str());
}

void write_trace_header()
{
    if (g_header_written.exchange(true)) {
        return; // Header already written
    }
    std::string trace_filename = determine_trace_filename("");
    if (trace_filename.empty()) {
        return; // No trace file
    }
    // Check if file exists and is non-empty
    bool file_exists = std::filesystem::exists(trace_filename);
    bool file_empty = true;
    if (file_exists) {
        std::ifstream file(trace_filename);
        file_empty = file.peek() == std::ifstream::traits_type::eof();
    }
    // Only write header if file is new or empty
    if (!file_exists || file_empty) {
        pid_t pid = getpid();
        // Try to detect ROS namespace from environment
        std::string ros_namespace = "";
        const char* env_namespace = std::getenv("ROS_NAMESPACE");
        if (env_namespace != nullptr) {
            ros_namespace = std::string(env_namespace);
        }
        // Get the command line with arguments
        std::string cmd_line = "";
        std::ifstream cmdline("/proc/self/cmdline");
        if (cmdline.is_open()) {
            std::string arg;
            bool first = true;
            while (std::getline(cmdline, arg, '\0')) {
                if (!first) {
                    cmd_line += " ";
                } else {
                    first = false;
                }
                cmd_line += arg;
            }
        }
        // Build header JSON
        std::string header_json = "{\"pid\":" + std::to_string(pid);
        if (!ros_namespace.empty()) {
            header_json += ",\"ros_namespace\":\"" + ros_namespace + "\"";
        }
        if (!cmd_line.empty()) {
            header_json += ",\"command_line\":\"" + cmd_line + "\"";
        }
        header_json += "}";
        // Write header directly to file
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);
        FILE *fp = fopen(trace_filename.c_str(), "a");
        if (fp != nullptr) {
            fputs("# ", fp);
            fputs(header_json.c_str(), fp);
            fputs("\n", fp);
            fflush(fp);
            fclose(fp);
        }
    }
}

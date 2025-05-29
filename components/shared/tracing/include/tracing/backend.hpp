#ifndef TRACING_BACKEND_HPP
#define TRACING_BACKEND_HPP

#include <chrono>
#include <string>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <variant>

namespace tracing {

typedef std::chrono::high_resolution_clock::time_point timestamp_t;

// inspired by spdlog
struct SourceLoc
{
    constexpr SourceLoc() = default;
    constexpr SourceLoc(const char *filename_in, int line_in, const char *funcname_in)
        : filename{filename_in}
        , line{line_in}
        , funcname{funcname_in}
    {}
    constexpr SourceLoc(const SourceLoc &other)
        : filename{other.filename}
        , line{other.line}
        , funcname{other.funcname}
    {}
    constexpr bool empty() const noexcept
    {
        return line == 0;
    }
    const char *filename{nullptr};
    int line{0};
    const char *funcname{nullptr};
}; // end of struct SourceLoc

class FunctionRecord
{
public:
    FunctionRecord(SourceLoc loc);
    ~FunctionRecord();
    // specializations for basic types
    void add_input(std::string const &varname, int value);
    void add_input(std::string const &varname, float value);
    void add_input(std::string const &varname, double value);
    void add_input(std::string const &varname, bool value);
    void add_input(std::string const &varname, std::string const &value);
    void add_output(std::string const &varname, int value);
    void add_output(std::string const &varname, float value);
    void add_output(std::string const &varname, double value);
    void add_output(std::string const &varname, bool value);
    void add_output(std::string const &varname, std::string const &value);
    // also support objects with operator<<
    template <typename T>
    std::enable_if_t<std::is_convertible_v<T, std::ostream&>, void>
    add_input(std::string const &varname, T const &value) {
        if constexpr (std::is_convertible_v<T, std::ostream&>) {
            std::ostringstream oss;
            oss << value;
            add_input(varname, oss.str());
        } else {
            throw std::runtime_error("cannot convert input variable " + varname + " to string");
        }
    }
    template <typename T>
    std::enable_if_t<std::is_convertible_v<T, std::ostream&>, void>
    add_output(std::string const &varname, T const &value) {
        if constexpr (std::is_convertible_v<T, std::ostream&>) {
            std::ostringstream oss;
            oss << value;
            add_output(varname, oss.str());
        } else {
            throw std::runtime_error("cannot convert output variable " + varname + " to string");
        }
    }
    // flushers
    void flush_input();
    void flush_output(timestamp_t const &timestamp);
private:
    std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> input_data_;
    std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> output_data_;
    SourceLoc loc_;
    std::string convert_to_json(std::vector<std::pair<std::string, std::variant<int, double, bool, std::string>>> const &data);
    timestamp_t timestamp_start_;
}; // end of class FunctionRecord

} // end of namespace tracing

#endif // TRACING_BACKEND_HPP

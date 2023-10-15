/**
 *  @file
 *  @brief   Generic string functions
 *  @curator Jürge van Eijck
 */
#ifndef STRING_UTIL_H
#define STRING_UTIL_H
#include <string>
#include <vector>

class StringUtil {
public:
	static std::vector<std::string> split(const std::string& str, const std::string& delimiters);
	static std::string convertNumberToString(int number);
	static std::string convertNumberToString(double number, const std::string& format);
	static std::string SkullString();
	static std::string getLocalTimeString();
	static double convertTimeStringToSeconds(const std::string& rTimeString);  /* input-format: mm:ss.ms */
	static std::string convertSecondsToString(double t); /* output-format: mm:ss.ms */
};

#endif

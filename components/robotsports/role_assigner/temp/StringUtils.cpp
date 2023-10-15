/**
 *  @file
 *  @brief   Generic string functions
 *  @curator Jürge van Eijck
 */
#include "StringUtils.h"

#include <iostream>
#include <string>
#include <ostream>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <time.h>
#include <cmath>
#include <iomanip>

using namespace std;

// split str in vector of string, using delimiters as separator
vector<string> StringUtil::split(const string& str, const string& delimiters) {
    vector<string> v;
    string::size_type start = 0;
    auto pos = str.find_first_of(delimiters, start);
    while(pos != string::npos) {
        if(pos != start) // ignore empty tokens
            v.emplace_back(str, start, pos - start);
        start = pos + 1;
        pos = str.find_first_of(delimiters, start);
    }
    if(start < str.length()) // ignore trailing delimiter
        v.emplace_back(str, start, str.length() - start); // add what's left of the string
    return v;
}

std::string StringUtil::getLocalTimeString()
{
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "%T", &tstruct);  // time as HH:MM:SS
	return buf;
}


std::string StringUtil::convertNumberToString(int number) {
	std::string String = static_cast<std::ostringstream*>( &(std::ostringstream() << number) )->str();
	return String;
}

std::string StringUtil::convertNumberToString(double number, const std::string& format) {
	char buffer[256];
	sprintf(buffer, format.c_str(), number);
	std::string String = buffer;
	return String;
}


std::string StringUtil::SkullString() {
	std::string result =
			"                  uuuuuuu \n " \
			"             uu$$$$$$$$$$$uu \n " \
			"          uu$$$$$$$$$$$$$$$$$uu \n " \
			"         u$$$$$$$$$$$$$$$$$$$$$u \n " \
			"        u$$$$$$$$$$$$$$$$$$$$$$$u \n " \
			"       u$$$$$$$$$$$$$$$$$$$$$$$$$u \n " \
			"       u$$$$$$$$$$$$$$$$$$$$$$$$$u \n " \
			"       u$$$$$$\"   \"$$$\"   \"$$$$$$u \n " \
			"       u$$$$$$\"   \"$$$\"   \"$$$$$$u \n " \
			"       \"$$$$\"      u$u       $$$$\" \n " \
			"        $$$u       u$u       u$$$ \n " \
			"        $$$u      u$$$u      u$$$ \n " \
			"         \"$$$$uu$$$   $$$uu$$$$\" \n " \
			"          \"$$$$$$$\"   \"$$$$$$$\" \n " \
			"            u$$$$$$$u$$$$$$$u \n " \
			"             u$\"$\"$\"$\"$\"$\"$u \n " \
			"  uuu        $$u$ $ $ $ $u$$       uuu \n " \
			" u$$$$        $$$$$u$u$u$$$       u$$$$ \n "  \
			"  $$$$$uu      \"$$$$$$$$$\"     uu$$$$$$ \n " \
			"u$$$$$$$$$$$uu    \"\"\"\"\"    uuuu$$$$$$$$$$ \n " \
			"$$$$\"\"\"$$$$$$$$$$uuu   uu$$$$$$$$$\"\"\"$$$ \" \n " \
			"\"\"\"      \"\"$$$$$$$$$$$uu \"\"$\"\"\" \n " \
			"           uuuu \"\"$$$$$$$$$$uuu \n " \
			"  u$$$uuu$$$$$$$$$uu \"\"$$$$$$$$$$$uuu$$$ \n " \
			"  $$$$$$$$$$\"\"\"\"           \"\"$$$$$$$$$$$ \" \n " \
			"  \"$$$$$\"                      \"\"$$$$\"\" \n " \
			"     $$$\"                         $$$$\" \n ";
	return result;
}

double StringUtil::convertTimeStringToSeconds(const std::string& rTimeString) {  /* input-format: mm:ss.ms */
	double minutes = std::stod(rTimeString.substr(0,2)) * 60.0;
	double seconds = std::stod(rTimeString.substr(3,rTimeString.length()-3));
	return minutes + seconds;
}

std::string StringUtil::convertSecondsToString(double t) {  /* output-format: mm:ss.ms */
	int minutes = t / 60;
	double seconds = t - (minutes * 60);
	std::string t_str =  StringUtil::convertNumberToString(minutes, "%02.0f") + ":" + StringUtil::convertNumberToString(seconds, "%05.2f");
	return t_str;
}


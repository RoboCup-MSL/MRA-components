/**
 *  @file
 *  @brief  tool for testing teamplanner with xml files
 *  @curator Jürge van Eijck
 */
#ifndef ROLEASSIGNER_REGRESSION_CHECKER_H
#define ROLEASSIGNER_REGRESSION_CHECKER_H 1
#include <string>

int validate_regression(const std::string& regression_dir, const std::string& output_dir);

#endif


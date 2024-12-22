/**
 *  @file
 *  @brief   main file for role assinger regreession checker
 *  @curator Jürge van Eijck
 */
#include <string>
#include <iostream>

#include "roleassigner_regression_checker.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "use program: " << argv[0] << " <regression-directory> <compare-to-direcotry> " << endl;
        return -1;
    }
    else {
        string regression_dir = argv[1];
        string output_dir = argv[2];

        validate_regression(regression_dir, output_dir);
    }
}

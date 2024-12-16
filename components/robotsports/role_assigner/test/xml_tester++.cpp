/**
 *  @file
 *  @brief   main file for testing the xml planner
 *  @curator Jürge van Eijck
 */
#include <iostream>
#include <cstdio>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <wait.h>
#include <math.h>
#include <signal.h>

#include "xmlRoleAssigner.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    try {
        // Arguments will be stored here
        if (argc != 2) {
            cout << "usage:" << endl;
            cout << "\t" << argv[0] << " <input-file>" << endl;

        }
        std::string input_file = argv[1];
        // call xml planner with the received parameters
        role_assigner_with_xml_input(input_file);
    }
    catch (std::exception & e)
    {
        cerr << "Exception:: " << e.what() << endl << flush;
    }
    catch (...) {
        cerr << "General Exception in " << argv[0] << endl << flush;
    }
}

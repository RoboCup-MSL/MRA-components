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
#include <boost/program_options.hpp>

#include "xmlRoleAssigner.hpp"

namespace po = boost::program_options;
using namespace std;


int main(int argc, char *argv[]) {
    try {
        // Arguments will be stored here
        std::string input_file;

        // Configure options here
        po::options_description desc ("Allowed options");
        desc.add_options ()
                ("help,h", "print usage message")
                ("input-file,i", po::value(&input_file), "Input file");

        // Parse command line arguments
        po::variables_map vm;
        po::store (po::command_line_parser (argc, argv).options (desc).run (), vm);
        po::notify (vm);
        // Check if --input is missing or --help is given
        if (vm.count ("help") || !vm.count ("input-file")) {
            std::cerr << desc << "\n";
            return 1;
        }

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

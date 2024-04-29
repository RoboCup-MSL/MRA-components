#!/bin/sh -x
# Run with valgrind to see if runtime is ok.
# compile with -g for valgrind with line numbers
# check output of valgrind for "Exit program on first error (--exit-on-first-error=yes)" if runnning full_regression.sh
valgrind --leak-check=full --track-origins=yes --dsymutil=yes --error-exitcode=42 --exit-on-first-error=yes ~/MRA-components/build/components/robotsports/role_assigner/xml_test --input-file $1 #--runs-needed 1 

#~/MRA-components/build/components/robotsports/role_assigner/bin/xml_test --input-file $1


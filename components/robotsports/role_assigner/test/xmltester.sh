#!/bin/sh
# Run with valgrind to see if runtime is ok.
# compile with -g for valgrind with line numbers
# check output of valgrind for "Exit program on first error (--exit-on-first-error=yes)" if runnning full_regression.sh
# catch valgrind output  my-command > output-file.txt 2>&1



# check number of arguments if 2
# 
RUN_VALGRIND=0
INPUTFILE=$1

# --input-file $1

if [ "$#" -eq "2" ]
then
    if [ "$1" = "-v" ]
    then
        RUN_VALGRIND=1
	INPUTFILE=$2
    fi
fi 


if [ $RUN_VALGRIND -eq "1" ] 
then
# argument 1 : -v
# argument 2 : <input-file>
    valgrind --leak-check=full --track-origins=yes --dsymutil=yes --error-exitcode=42 --exit-on-first-error=yes ~/MRA-components/build/components/robotsports/role_assigner/xml_test --input-file $INPUTFILE 
else
    ~/MRA-components/build/components/robotsports/role_assigner/xml_test --input-file $INPUTFILE
fi

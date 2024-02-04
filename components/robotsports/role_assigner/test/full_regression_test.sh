./clean_tmp_files.sh
./run_all_tests.sh
$ROBOTSPORTS/bin/planner_regression_checker
if ls diff*.svg 1> /dev/null 2>&1; then
	# show first difference
	eog `ls -t1 diff*.svg | head -n1`
fi


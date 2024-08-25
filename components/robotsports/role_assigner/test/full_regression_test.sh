./clean_tmp_files.sh
./run_all_tests.sh
$ROBOTSPORTS/bin/planner_regression_checker
if ls compare_old/*.svg 1> /dev/null 2>&1; then
	python3 ./create_regression_suite_difference_html_page.py
	firefox regression_suite_difference.html&
fi


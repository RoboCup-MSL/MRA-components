./clean_tmp_files.sh
./run_all_tests.sh
~/MRA-components/build/components/robotsports/role_assigner/roleassigner_regression_checker ~/MRA-components/components/robotsports/role_assigner/testdata/regression ~/MRA-components/components/robotsports/role_assigner/test/output_team
if ls compare_old/*.svg 1> /dev/null 2>&1; then
	python3 ./create_regression_suite_difference_html_page.py
	firefox regression_suite_difference.html&
fi


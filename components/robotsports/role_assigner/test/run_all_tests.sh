echo clearing temp files
./clean_tmp_files.sh
echo excuting script: "$0"
echo GlobalPath planner scripts
./test_globalpath_planner.sh
echo Teamplanner scripts
./test_attacksupport.sh
./test_ballplayer.sh
./test_dropped_ball.sh
./test_empty.sh
./test_freekick.sh
./test_issues.sh
./test_kickoff.sh
./test_msa.sh
./test_no_ball.sh
./test_no_opponent.sh
./test_penalty_area_timeout.sh
./test_penalty_ingame.sh
./test_penalty_shootout.sh
./test_priority_blocker.sh
./test_team_planner.sh
./test_throwin.sh
./test_auto_assign_keeper.sh
./test_tps.sh
./test_european_open_2022.sh


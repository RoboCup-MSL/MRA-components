This directory contain code for the vectorize path planner.
More info: Jurge van Eijck
It started with the translation of the latest java path planner into C++.

Documentation:
==============
More information about the planner can be found at :
- http://git.cybervalley.nl/robot-sports/robotsports/wikis/planner
- http://git.cybervalley.nl/robot-sports/robotsports/wikis/dynamic_role_assignment

REMAINING WORK
==============
To organise the remaining work, tasks have a prio.
Prio-1: needed for first game 
Prio-2: needed for World Cup 
Prio-3: Quick improvement
Prio-4: significant improvement but a lot of work

A lot of work must be done.

- filter away points in goal-area and Penalty-Area
	- Prio-2: current implementation works, but can lead to yellow cards, effort estimation: 6 hours. Pass through is allowed or first 10 seconds, if no other player is in penalty area
- allow point inside safety area
	- Prio-2: current implementation works, but is fast improvement wrt original implementation, effort estimation: 4 hours

- handle path of other teammates (dynamic and guess opponent plans)
	- ON ROBOT: Improve plans on robot by estimate the path of the opponent
		- Prio-4: effort 30 hours. 
	
- Use I-controlBall-flag for assign attacker role (dribble).
	- Prio-1: needed for work on robot, waiting for interfaces, effort 3 hours
	
- Assign player defending: X-dist and Y-dist must be weighted, current only distance taken into account.
	- working version present, can be improved 
	- Prio-3 

- Currently most threading player is player closest to ball. optional first defend player closest to ball between ball and our goal (automatical detection)
	- working version present, can be improved 
	- Algorithm to decide what is most dangerous oppoent. 
		- full automatic: Prio-4: Effort estimation: 30 hours 
		- via option: Prio-2: Effort estimation: 3 hours 
 
	
- Get settings from options (shared memory)
	- at least the following parameters 
		- default options must be filled in.
	- wait for interfaces, 3 hours work
	- Prio-2, current implementation works, but requires new code for modification
	
- add dialog to GUI for team strategy (options)
	- Prio-3: options are defined, waiting for interfaces
	
- TESTING:
	- create test with moving objects.

- Prepare enter game for substitute
	- prio-2: effort 3 hours
- keep distance from ball when a restart again is given. (rules compliance)
	- prio-2: testing, testing. effort 6 hours (otherwise yellow cards)

NEW:
===
	- improve calculationTime via option: calculateAllPaths (y|n)
	- zone-defense: improve formulas for better positioning.



TIP:
====
Check on memory leaks:
valgrind --leak-check=yes ./vector_test 1> memory_leaking.txt 2> memory_leaking.txt


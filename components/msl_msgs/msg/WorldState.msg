# WorldState.msg, as produced by WorldModel (or simulator)

# Absolute timestamp
builtin_interfaces/Time time

# Self (robot)
msl_msgs/Player robot

# Ball (allow maximum one ball in the field/test; if there are more, then WorldModel should provide the closest one)
msl_msgs/Ball ball

# Teammates (a mix of robots and humans)
msl_msgs/Player[] teammates

# Opponents (a mix of robots and humans)
msl_msgs/Player[] opponents

# Optional entities (like a black box or suitcase for testing, some kind of forbidden zone)
Obstacle[] obstacles

# TODO: goal posts

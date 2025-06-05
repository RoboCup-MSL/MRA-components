# Action, as produced by Teamplay (or test software), consumed by ActionPlanning

uint8 actiontype # enum ActionType
bool grab # enable ballhandlers
geometry_msgs/Pose targetposition # in FCS, depending on the action type, can be a target for the robot or the ball
geometry_msgs/Twist targetvelocity
float32 radius
int32 motiontype # fast/slow/accurate (alternative: service call?)

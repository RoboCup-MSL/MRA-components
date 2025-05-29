# Action, as produced by Teamplay (or test software), consumed by ActionPlanning

uint8 actiontype # enum ActionType
bool grab # enable ballhandlers
msl_msgs/Target target # in FCS, depending on the action, can be a target for the robot or the ball
float32 radius
int32 motiontype # fast/slow/accurate (alternative: service call?)

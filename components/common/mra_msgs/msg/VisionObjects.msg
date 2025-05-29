# a set of vision observations, typically produced by YOLO, sent to WorldModel

int32 frame # counter
int32 robot
builtin_interfaces/Time timestamp
mra_msgs/VisionObject[] objects

# Targets.msg: targets as produced by ActionPlanning, which should be abstract enough, not hardware-specific


bool grab                       # ball grabber on/off
msl_msgs/MovementTarget move    # target pose in FCS, unsafe, to be handled by PathPlanning / VelocityControl
msl_msgs/ShootingTarget shoot   # where the ball should be kicked
msl_msgs/KeeperTarget keeper    # where the hands should go

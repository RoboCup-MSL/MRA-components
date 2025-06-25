# Targets.msg: targets as produced by ActionPlanning, which should be abstract enough, not hardware-specific


bool grab                              # ball grabber on/off
mra_common_msgs/MovementTarget move    # target pose in FCS, unsafe, to be handled by PathPlanning / VelocityControl
mra_common_msgs/ShootingTarget shoot   # where the ball should be kicked
mra_common_msgs/HandsTarget hands      # where the hands should go

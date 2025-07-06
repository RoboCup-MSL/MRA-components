# Summary

WorldModel is the central perception and state estimation component responsible for
providing the `/world_state` topic of type `WorldState.msg`. It fuses data from
multiple sources to create a comprehensive, unified view of the game state, including:

* **Self localization**: Robot pose estimation by fusing wheel odometry with vision-based landmarks
* **Ball tracking**: Multi-robot collaborative ball state estimation with 3D position and velocity
* **Player tracking**: Teammate and opponent/obstacle detection and tracking
* **Team coordination**: Data exchange and synchronization between team members

# Operational notes

It is decided to initialize playing **forward**. This means that when a robot is put on
the field, it will choose its playing direction based on the goal it is facing.

# Status

MVP implementation with basic structure. Active development is happening in `falcons/code`
branch `world_model_refactoring` ("worldmodel2") by Florent. At some point it should be migrated to this MRA structure.

# Interface Details

## Input Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/vision` | `mra_common_msgs/VisionObjects` | Vision candidates with confidence, in Robot Coordinate System (RCS) |
| `/feedback` | `falcons_msgs/Feedback` | Odometry, buttons, and sensor feedback from hardware/embedded systems |
| TBD | TBD | Shared perception data from teammates (planned) |

## Output Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/world_state` | `mra_common_msgs/WorldState` | Complete world state including self, ball, players, and obstacles |
| TBD | TBD | Local perception data shared with teammates (planned) |

# Designs

## Localization robustness

Localization must be able to deal with a lack of, or temporarily incorrect absolute vision
data. It may initialize on the TTP (substitution) area, where insufficient landmarks are
visible. Its vision to landmarks may also be blocked by other robots.

Key robustness requirements:
- **Temporal consistency**: Maintain pose estimates over time even with sensor gaps
- **Multiple information sources**: Combine odometry, goal posts, field lines, and team data
- **Orientation disambiguation**: Detect and resolve 180° orientation errors ("flipping")
- **Confidence estimation**: Provide uncertainty measures for downstream components

## Sub-components

* **LocalizationLandmarks**: fit candidate field pose (FCS) from observed vision landmarks (RCS), which are typically goalposts and/or white line pixels
* **LocalizationFusion**: fuse odometry with absolute vision field pose candidates
* **BallTracking**: determine ball location and velocity by combining 3d observations from multiple robots
* **PlayerTracking**: track teammates, humans, opponents and obstacles detected through vision

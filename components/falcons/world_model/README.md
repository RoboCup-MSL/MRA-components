# Summary

WorldModel is responsible for providing the `/world_state` topic of type `WorldState.msg`, which includes:
* self localization, fusing wheel odometry and vision landmark observations
* teammembers
* ball tracking, including velocity, 3d
* opponent / obstacle / human tracking

# Status

MVP. Active development is happening in `falcons/code` branch `worldmodel2`. When the time is right, it should be ported here.

# Operational & environment notes

It is decided to initialize playing **forward**. This means that when a robot is put on the field, it will choose its playing direction based on the goal it is facing.

Localization must be able to deal with a lack of, or temporarily incorrect absolution vision data. It may initialize on the TTP (substitution) area, where insufficient landmarks are visible. Its vision to landmarks may also be blocked by other robots.

# Interface details

## Inputs

* `/vision`: vision candidates, interpreted coordinates in RCS, with confidence indication
* `/feedback`: buttons and odometry
* TBD: teammembers data exchange, via topics or custom? need to include vision data also somehow

## Outputs

* `/world_state`
* TBD: teammembers data exchange

# Design

## Sub-components

* LocalizationLandmarks: fit candidate field pose (FCS) from observed vision landmarks (RCS), which are typically goalposts and/or white line pixels
* LocalizationFusion: fuse odometry with absolute vision field pose candidates
* TeamMembers: administrative component to disclose friendly data in to `/world_state`
* BallTracking: determine ball location and velocity by combining 3d observations from multiple robots
* ...

NOTE: some of these sub components will probably move to their dedicated library component later, for now just use a .hpp, .cpp and class name as specified.


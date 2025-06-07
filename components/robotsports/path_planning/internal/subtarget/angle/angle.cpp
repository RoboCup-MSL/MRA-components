#include "angle.hpp"

// A simple wrap function to mimic setpoint.wrap
// Wraps angle to be within [-PI, PI) relative to current_setpoint_angle
double wrap_angle(double angle, double current_setpoint_angle) {
    double diff = angle - current_setpoint_angle;
    // Normalize diff to be within [-PI, PI)
    diff = fmod(diff + M_PI, 2 * M_PI);
    if (diff < 0) {
        diff += 2 * M_PI;
    }
    return current_setpoint_angle + diff - M_PI;
}

double set_angle(const GlobalData& d) {
    double angle;

    // check skillID:
    // 0 - move
    // 1 - dribble
    // 2 - aim
    // 3 - kick
    // 4 - shield
    // 5 - intercept

    if (d.input.robot.skillID == 1) { // dribble
        // To call dribble, we need to populate a DribbleData struct.
        // This is a common pattern when translating from dynamic languages to static ones.
        DribbleData dribble_data;
        dribble_data.setpoint.v = d.setpoint.v;
        dribble_data.setpoint.p = d.setpoint.p;
        angle = dribble(dribble_data);
    } else if (d.input.robot.skillID == 2) { // aim
        AimData aim_data;
        aim_data.target.p = d.target.p;
        aim_data.setpoint.p = d.setpoint.p;
        aim_data.subtarget.p = d.subtarget.p;
        angle = aim_at_target(aim_data);
    } else if (d.input.robot.skillID == 4) { // shield
        ShieldData shield_data;
        shield_data.input.obstacles.p = d.input.obstacles.p;
        shield_data.input.obstacles.active = d.input.obstacles.active;
        shield_data.setpoint.p = d.setpoint.p;
        shield_data.setpoint.v = d.setpoint.v;
        shield_data.subtarget.p = d.subtarget.p;
        angle = shield(shield_data);
    } else {
        angle = d.target.p[2]; // Equivalent to d['target']['p'][2]
    }

    // wrap around current setpoint using the spg wrap function
    // Assuming d['setpoint']['p'][3] was meant to be d['setpoint']['p'][2] (the angle itself)
    // or if p[3] was intended for a different angle reference.
    // Given 0-based indexing mentioned in Python comment, p[2] is the angle.
    angle = wrap_angle(angle, d.setpoint.p[2]); // Using p[2] for the current angle.
    return angle;
}
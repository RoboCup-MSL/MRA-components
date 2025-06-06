#include <cmath>

double wrap(double angle, double angle_setpoint) {
    angle = fmod(angle - angle_setpoint + M_PI, 2 * M_PI) + angle_setpoint - M_PI;
    return angle;
}
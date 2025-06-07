#include "wrap.hpp"

double wrap(double angle, double reference) {
    // This implementation matches the Python code's logic for angle wrapping
    // to keep `angle` within `reference - pi` and `reference + pi`.
    while (angle > reference + M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < reference - M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}
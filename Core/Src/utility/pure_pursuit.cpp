#include "pure_pursuit.h"

float calculateCurvature(const Vector &position, const float &orientation, const Vector &target)
{
    // Calculate the relative position to the target point
    float dx = *target.x - *position.x;
    float dy = *target.y - *position.y;

    // Find angle between heading and target
    float target_angle = atan2(dy, dx);
    float angle_diff = target_angle - orientation;

    // Calculate Curvature
    return 2.0f * sin(angle_diff) / hypot(dx, dy);
}

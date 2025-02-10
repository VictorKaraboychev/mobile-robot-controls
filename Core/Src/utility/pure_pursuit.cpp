#include "pure_pursuit.h"

float calculateCurvatureRadius(const State &robot, const Vector &target)
{
    // Calculate the relative position to the target point
    float dx = *target.x - *robot.position.x;
    float dy = *target.y - *robot.position.y;

    // Find angle between heading and target
    float target_angle = atan2(dy, dx);
    float angle_diff = target_angle - robot.orientation;

    // Calculate Curvature
    return 2.0f * sin(angle_diff) / hypot(dx, dy);
}

float calculateSpeedRatio(float curvature_radius, float wheel_distance)
{
    return curvature_radius / wheel_distance;
}
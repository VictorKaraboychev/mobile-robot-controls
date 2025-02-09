#include <cmath>
#include "sensors.h"
#include "control.h"
#include "vector.h"

float calculateCurvatureRadius(const State& robot, const Vector& target) {
    // Calculate the relative position to the target point
    float dx = *target.x - *robot.position.x;
    float dy = *target.y - *robot.position.y;

    // Find angle between heading and target
    float target_angle = atan2(dy, dx);
    float angle_diff = target_angle - robot.orientation;

    //Calculate Curvatire
    return 2*sin(angle_diff) / hypot(dx,dy);
}

float calculateSpeedRatio(float cuvature_radius){
    return cuvature_radius / WHEEL_DISTANCE;
}
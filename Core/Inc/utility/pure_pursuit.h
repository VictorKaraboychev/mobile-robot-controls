#ifndef __PURE_PURSUIT_H__
#define __PURE_PURSUIT_H__

#include <cmath>

#include "utility/vector.h"

#include "control.h"

float calculateCurvatureRadius(const State &robot, const Vector &target);
float calculateSpeedRatio(float curvature_radius, float wheel_distance);

#endif /* __PURE_PURSUIT_H__ */
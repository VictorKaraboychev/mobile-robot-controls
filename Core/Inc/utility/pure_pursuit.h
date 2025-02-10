#ifndef __PURE_PURSUIT_H__
#define __PURE_PURSUIT_H__

#include <cmath>

#include "utility/vector.h"

#include "control.h"

float calculateCurvature(const State &robot, const Vector &target);

#endif /* __PURE_PURSUIT_H__ */
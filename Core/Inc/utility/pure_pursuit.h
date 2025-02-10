#ifndef __PURE_PURSUIT_H__
#define __PURE_PURSUIT_H__

#include <cmath>

#include "utility/vector.h"

float calculateCurvature(const Vector &position, const float &orientation, const Vector &target);

#endif /* __PURE_PURSUIT_H__ */
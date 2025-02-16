#ifndef __PURE_PURSUIT_H__
#define __PURE_PURSUIT_H__

#include <cmath>
#include <Eigen/Dense>

template <typename DataType = float>
class PurePursuit
{
	public:
	// Define a 2D vector
	using Vector2 = Eigen::Vector<DataType, 2>;

	// Calculate the curvature of the path at a given point and orientation towards a target point
	static DataType CalculateCurvature(const Vector2 &position, const DataType &orientation, const Vector2 &target);
};

template <typename DataType>
DataType PurePursuit<DataType>::CalculateCurvature(const Vector2 &position, const DataType &orientation, const Vector2 &target)
{
    // Calculate the relative position to the target point
    Eigen::Vector2f d = target - position;

    // Find angle between heading and target
    DataType target_angle = atan2(d[1], d[0]);
    DataType angle_diff = target_angle - orientation;

    // Calculate Curvature
    return 2.0f * sin(angle_diff) / hypot(d[0], d[1]);
}


#endif /* __PURE_PURSUIT_H__ */
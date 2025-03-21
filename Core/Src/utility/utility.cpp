#include "utility/utility.h"

Eigen::Matrix3f eulerToMatrix(Eigen::Vector3f orientation)
{
	Eigen::AngleAxisf rollAngle(orientation[0], Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(orientation[1], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(orientation[2], Eigen::Vector3f::UnitZ());

	return (rollAngle * pitchAngle * yawAngle).matrix();
}
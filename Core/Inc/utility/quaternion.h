#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "vector.h"

class Quaternion
{
public:
	Quaternion();
	Quaternion(const float x, const float y, const float z, const float w);
	Quaternion(const float pitch, const float roll, const float yaw);
	Quaternion(const Quaternion &q);
	~Quaternion();

	float x;
	float y;
	float z;
	float w;

	Quaternion &operator=(const Quaternion &q);

	Quaternion operator+(const Quaternion &q) const;
	Quaternion operator-(const Quaternion &q) const;
	Quaternion operator*(const Quaternion &q) const;
	Quaternion operator*(const float &s) const;
	Quaternion operator/(const float &s) const;

	void operator+=(const Quaternion &q);
	void operator-=(const Quaternion &q);
	void operator*=(const Quaternion &q);
	void operator*=(const float &s);
	void operator/=(const float &s);

	float magnitude() const;
	Quaternion normalize() const;
	Quaternion conjugate() const;
	Quaternion inverse() const;

	Vector rotate(const Vector &v) const;
	Vector toEuler() const;

	void print() const;
};

#endif /* __QUATERNION_H__ */
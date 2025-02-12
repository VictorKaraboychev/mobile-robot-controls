#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <math.h>
#include <stdint.h>
#include <initializer_list>
#include <string.h>
#include <stdio.h>

#define VECTOR3_ROTATION(a, b, c) \
	Vector { cos(a) * cos(b), sin(a) * cos(b), sin(b) }
#define VECTOR2_ROTATION(theta) \
	Vector { cos(theta), sin(theta) }

class Vector
{
public:
	Vector();
	Vector(const uint8_t size);
	Vector(float *data, const uint8_t size);
	Vector(const std::initializer_list<float> &list);
	Vector(const Vector &v);
	~Vector();

	float *x;
	float *y;
	float *z;

	uint8_t size() const;

	Vector set(uint8_t index, float value);
	float get(uint8_t index) const;
	Vector getSubVector(uint8_t start, uint8_t end) const;

	float &operator[](uint8_t index);
	float operator[](uint8_t index) const;

	float &operator()(uint8_t index);
	float operator()(uint8_t index) const;

	Vector append(float value);
	Vector append(const Vector &v);
	Vector remove(uint8_t index);

	Vector &operator=(const Vector &v);
	Vector operator-() const;

	Vector operator+(const Vector &v) const;
	Vector operator-(const Vector &v) const;
	Vector operator*(const Vector &v) const;
	Vector operator*(const float &s) const;
	Vector operator/(const float &s) const;

	void operator+=(const Vector &v);
	void operator-=(const Vector &v);
	void operator*=(const Vector &v);
	void operator*=(const float &s);
	void operator/=(const float &s);

	float magnitude() const;
	Vector normalize() const;

	float dot(const Vector &v) const;
	Vector cross(const Vector &v) const;

	void print() const;

	static Vector Zero(uint8_t size);
	static Vector One(uint8_t size);
	static Vector Exp(const Vector &v);

private:
	uint8_t _size;
	float *_data;

	void _updatexyz();
};

#endif // __VECTOR_H__
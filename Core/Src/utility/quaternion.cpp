#include "quaternion.h"

// Quaternion class implementation

Quaternion::Quaternion()
{
	this->x = 0;
	this->y = 0;
	this->z = 0;
	this->w = 0;
}

Quaternion::Quaternion(float x, float y, float z, float w)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

Quaternion::Quaternion(float pitch, float roll, float yaw)
{
	float cy = cosf(yaw * 0.5f);
	float sy = sinf(yaw * 0.5f);
	float cr = cosf(roll * 0.5f);
	float sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f);
	float sp = sinf(pitch * 0.5f);

	this->x = cy * sr * cp - sy * cr * sp;
	this->y = cy * cr * sp + sy * sr * cp;
	this->z = sy * cr * cp - cy * sr * sp;
	this->w = cy * cr * cp + sy * sr * sp;
}

Quaternion::Quaternion(const Quaternion &q)
{
	this->x = q.x;
	this->y = q.y;
	this->z = q.z;
	this->w = q.w;
}

Quaternion::~Quaternion()
{
}

Quaternion &Quaternion::operator=(const Quaternion &q)
{
	this->x = q.x;
	this->y = q.y;
	this->z = q.z;
	this->w = q.w;

	return *this;
}

Quaternion Quaternion::operator+(const Quaternion &q) const
{
	return Quaternion(this->x + q.x, this->y + q.y, this->z + q.z, this->w + q.w);
}

Quaternion Quaternion::operator-(const Quaternion &q) const
{
	return Quaternion(this->x - q.x, this->y - q.y, this->z - q.z, this->w - q.w);
}

Quaternion Quaternion::operator*(const Quaternion &q) const
{
	return Quaternion(
		this->w * q.x + this->x * q.w + this->y * q.z - this->z * q.y, // x
		this->w * q.y - this->x * q.z + this->y * q.w + this->z * q.x, // y
		this->w * q.z + this->x * q.y - this->y * q.x + this->z * q.w, // z
		this->w * q.w - this->x * q.x - this->y * q.y - this->z * q.z  // w
	);
}

Quaternion Quaternion::operator*(const float &s) const
{
	return Quaternion(this->x * s, this->y * s, this->z * s, this->w * s);
}

Quaternion Quaternion::operator/(const float &s) const
{
	return Quaternion(this->x / s, this->y / s, this->z / s, this->w / s);
}

void Quaternion::operator+=(const Quaternion &q)
{
	this->x += q.x;
	this->y += q.y;
	this->z += q.z;
	this->w += q.w;
}

void Quaternion::operator-=(const Quaternion &q)
{
	this->x -= q.x;
	this->y -= q.y;
	this->z -= q.z;
	this->w -= q.w;
}

void Quaternion::operator*=(const Quaternion &q)
{
	*this = *this * q;
}

void Quaternion::operator*=(const float &s)
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	this->w *= s;
}

void Quaternion::operator/=(const float &s)
{
	this->x /= s;
	this->y /= s;
	this->z /= s;
	this->w /= s;
}

float Quaternion::magnitude() const
{
	return sqrtf(this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
}

Quaternion Quaternion::normalize() const
{
	return *this / this->magnitude();
}

Quaternion Quaternion::conjugate() const
{
	return Quaternion(-this->x, -this->y, -this->z, this->w);
}

Quaternion Quaternion::inverse() const
{
	return this->conjugate() / (this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);
}

Vector Quaternion::rotate(const Vector &v) const
{
	Quaternion qv(0, v[0], v[1], v[2]);
	Quaternion qr = *this * qv * this->inverse();

	return Vector{qr.y, qr.z, qr.w};
}

Vector Quaternion::toEuler() const
{
	Vector euler(3);

	float sinr_cosp = 2 * (this->w * this->x + this->y * this->z);
	float cosr_cosp = 1 - 2 * (this->x * this->x + this->y * this->y);
	euler[0] = atan2f(sinr_cosp, cosr_cosp);

	float sinp = 2 * (this->w * this->y - this->z * this->x);
	if (fabs(sinp) >= 1)
	{
		euler[1] = copysignf(M_PI / 2, sinp);
	}
	else
	{
		euler[1] = asinf(sinp);
	}

	float siny_cosp = 2 * (this->w * this->z + this->x * this->y);
	float cosy_cosp = 1 - 2 * (this->y * this->y + this->z * this->z);
	euler[2] = atan2f(siny_cosp, cosy_cosp);

	return euler;
}

void Quaternion::print() const
{
	printf("[%.4f, %.4f, %.4f, %.4f]\n", this->x, this->y, this->z, this->w);
}
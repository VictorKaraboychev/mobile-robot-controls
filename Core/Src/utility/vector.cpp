#include "vector.h"

// Vector class implementation

Vector::Vector()
{
	this->_data = NULL;
	this->_size = 0;
}

Vector::Vector(uint8_t size)
{
	this->_data = new float[size]{0};
	this->_size = size;

	this->_updatexyz();
}

Vector::Vector(float *data, uint8_t size)
{
	this->_data = new float[size];
	this->_size = size;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] = data[i];
	}

	this->_updatexyz();
}

Vector::Vector(std::initializer_list<float> data)
{
	this->_size = data.size();
	this->_data = new float[this->_size];

	uint8_t i = 0;
	for (const auto &value : data)
	{
		this->_data[i++] = value;
	}

	this->_updatexyz();
}

Vector::Vector(const Vector &v)
{
	this->_data = new float[v._size];
	this->_size = v._size;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] = v._data[i];
	}

	this->_updatexyz();
}

Vector::~Vector()
{
	if (this->_data != NULL)
	{
		delete[] this->_data;
	}
}

uint8_t Vector::size() const
{
	return this->_size;
}

Vector Vector::set(uint8_t index, float value)
{
	if (index < this->_size)
	{
		this->_data[index] = value;
	}
	return *this;
}

float Vector::get(uint8_t index) const
{
	return (index < this->_size) ? this->_data[index] : 0;
}

Vector Vector::getSubVector(uint8_t start, uint8_t end) const
{
	if (start >= this->_size || end >= this->_size || start > end)
	{
		throw "Invalid start or end index";
		return Vector();
	}

	Vector result(end - start + 1);

	for (uint8_t i = start; i <= end; i++)
	{
		result._data[i - start] = this->_data[i];
	}

	return result;
}

float &Vector::operator[](uint8_t index)
{
	return this->_data[index];
}

float Vector::operator[](uint8_t index) const
{
	return this->_data[index];
}

float &Vector::operator()(uint8_t index)
{
	return this->_data[index];
}

float Vector::operator()(uint8_t index) const
{
	return this->_data[index];
}

Vector Vector::append(float value)
{
	float *newdata = new float[this->_size + 1];

	for (uint8_t i = 0; i < this->_size; i++)
	{
		newdata[i] = this->_data[i];
	}

	newdata[this->_size] = value;

	if (this->_data != NULL)
	{
		delete[] this->_data;
	}

	this->_data = newdata;
	this->_size++;

	this->_updatexyz();

	return *this;
}

Vector Vector::append(const Vector &v)
{
	float *newdata = new float[this->_size + v._size];

	for (uint8_t i = 0; i < this->_size; i++)
	{
		newdata[i] = this->_data[i];
	}

	for (uint8_t i = 0; i < v._size; i++)
	{
		newdata[this->_size + i] = v._data[i];
	}

	if (this->_data != NULL)
	{
		delete[] this->_data;
	}

	this->_data = newdata;
	this->_size += v._size;

	this->_updatexyz();

	return *this;
}

Vector Vector::remove(uint8_t index)
{
	if (index >= this->_size)
	{
		return *this;
	}

	float *newdata = new float[this->_size - 1];

	for (uint8_t i = 0; i < index; i++)
	{
		newdata[i] = this->_data[i];
	}

	for (uint8_t i = index + 1; i < this->_size; i++)
	{
		newdata[i - 1] = this->_data[i];
	}

	if (this->_data != NULL)
	{
		delete[] this->_data;
	}

	this->_data = newdata;
	this->_size--;

	this->_updatexyz();

	return *this;
}

Vector &Vector::operator=(const Vector &v)
{
	if (this->_data != NULL)
	{
		delete[] this->_data;
	}

	this->_data = new float[v._size];
	this->_size = v._size;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] = v._data[i];
	}

	this->_updatexyz();

	return *this;
}

Vector Vector::operator-() const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = -this->_data[i];
	}

	return result;
}

Vector Vector::operator+(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] + v._data[i];
	}

	return result;
}

Vector Vector::operator-(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] - v._data[i];
	}

	return result;
}

Vector Vector::operator*(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] * v._data[i];
	}

	return result;
}

Vector Vector::operator*(const float &s) const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] * s;
	}

	return result;
}

Vector Vector::operator/(const float &s) const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] / s;
	}

	return result;
}

void Vector::operator+=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] += v._data[i];
	}
}

void Vector::operator-=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] -= v._data[i];
	}
}

void Vector::operator*=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] *= v._data[i];
	}
}

void Vector::operator*=(const float &s)
{
	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] *= s;
	}
}

void Vector::operator/=(const float &s)
{
	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] /= s;
	}
}

float Vector::magnitude() const
{
	float sum = 0;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		sum += this->_data[i] * this->_data[i];
	}

	return sqrt(sum);
}

Vector Vector::normalize() const
{
	float mag = magnitude();
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] / mag;
	}

	return result;
}

float Vector::dot(const Vector &v) const
{
	if (this->_size != v._size)
	{
		return 0;
	}

	float sum = 0;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		sum += this->_data[i] * v._data[i];
	}

	return sum;
}

Vector Vector::cross(const Vector &v) const
{
	if (this->_size != 3 || v._size != 3)
	{
		throw "Cross product is only defined for 3D vectors";
		return Vector();
	}

	Vector result(3);

	result._data[0] = this->_data[1] * v._data[2] - this->_data[2] * v._data[1]; // i = j * k - k * j
	result._data[1] = this->_data[2] * v._data[0] - this->_data[0] * v._data[2]; // j = k * i - i * k
	result._data[2] = this->_data[0] * v._data[1] - this->_data[1] * v._data[0]; // k = i * j - j * i

	return result;
}

void Vector::print() const
{
	char buffer[16];
	char *result = new char[this->_size * 32];

	strcpy(result, "[");

	for (uint8_t i = 0; i < this->_size; i++)
	{
		sprintf(buffer, "%.4f", this->_data[i]);
		strcat(result, buffer);

		if (i < this->_size - 1)
		{
			strcat(result, ", ");
		}
	}

	strcat(result, "]");

	printf("%s\n", result);
	delete[] result;
}

Vector Vector::zero(uint8_t size)
{
	return Vector(size);
}

Vector Vector::one(uint8_t size)
{
	Vector result(size);

	for (uint8_t i = 0; i < size; i++)
	{
		result._data[i] = 1;
	}

	return result;
}

Vector Vector::exp(const Vector &v)
{
	Vector result(v._size);

	for (uint8_t i = 0; i < v._size; i++)
	{
		result._data[i] = expf(v._data[i]);
	}

	return result;
}

void Vector::_updatexyz()
{
	uint8_t s = this->_size;

	if (s > 0)
	{
		this->x = this->_data + 0;
	}

	if (s > 1)
	{
		this->y = this->_data + 1;
	}

	if (s > 2)
	{
		this->z = this->_data + 2;
	}
}

#include "matrix.h"

// Matrix class implementation

Matrix::Matrix()
{
	this->_rows = 0;
	this->_cols = 0;
	this->_data = NULL;
}

Matrix::Matrix(const uint8_t size)
{
	this->_rows = size;
	this->_cols = size;

	this->_data = new float *[size];
	for (uint8_t i = 0; i < size; i++)
	{
		this->_data[i] = new float[size]{0};
	}
}

Matrix::Matrix(const uint8_t rows, const uint8_t cols)
{
	this->_rows = rows;
	this->_cols = cols;

	this->_data = new float *[rows];
	for (uint8_t i = 0; i < rows; i++)
	{
		this->_data[i] = new float[cols]{0};
	}
}

Matrix::Matrix(float **data, const uint8_t rows, const uint8_t cols)
{
	this->_rows = rows;
	this->_cols = cols;

	this->_data = new float *[rows];
	for (uint8_t i = 0; i < this->_rows; i++)
	{
		this->_data[i] = new float[cols];

		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] = data[i][j];
		}
	}
}

Matrix::Matrix(const std::initializer_list<std::initializer_list<float>> &data)
{
	this->_rows = data.size();
	this->_cols = data.begin()->size();

	// Allocate memory for the 2D array
	this->_data = new float *[this->_rows];
	for (uint8_t i = 0; i < this->_rows; ++i)
	{
		this->_data[i] = new float[this->_cols];
	}

	// Initialize the matrix with the provided data
	uint8_t i = 0;
	for (const auto &row : data)
	{
		if (row.size() != this->_cols)
		{
			throw "All rows must have the same number of columns";
			return;
		}

		uint8_t j = 0;
		for (const auto &value : row)
		{
			_data[i][j] = value;
			++j;
		}
		++i;
	}
}

Matrix::Matrix(const Matrix &m)
{
	this->_rows = m._rows;
	this->_cols = m._cols;

	this->_data = new float *[this->_rows];
	for (uint8_t i = 0; i < this->_rows; i++)
	{
		this->_data[i] = new float[this->_cols];

		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] = m._data[i][j];
		}
	}
}

Matrix::~Matrix()
{
	if (this->_data != NULL)
	{
		for (uint8_t i = 0; i < this->_rows; i++)
		{
			delete[] this->_data[i];
		}

		delete[] this->_data;
	}
}

uint8_t Matrix::rows() const
{
	return this->_rows;
}

uint8_t Matrix::cols() const
{
	return this->_cols;
}

Matrix Matrix::set(uint8_t row, uint8_t col, float value)
{
	if (row >= this->_rows || col >= this->_cols)
	{
		throw "Matrix index out of bounds";
		return *this;
	}

	this->_data[row][col] = value;

	return *this;
}

Matrix Matrix::setRow(uint8_t row, const Vector &v)
{
	if (row >= this->_rows || v.size() != this->_cols)
	{
		throw "Matrix dimensions do not match";
		return *this;
	}

	for (uint8_t i = 0; i < this->_cols; i++)
	{
		this->_data[row][i] = v[i];
	}

	return *this;
}

Matrix Matrix::setCol(uint8_t col, const Vector &v)
{
	if (col >= this->_cols || v.size() != this->_rows)
	{
		throw "Matrix dimensions do not match";
		return *this;
	}

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		this->_data[i][col] = v[i];
	}

	return *this;
}

Matrix Matrix::setSubMatrix(uint8_t row, uint8_t col, const Matrix &m)
{
	if (row + m._rows > this->_rows || col + m._cols > this->_cols)
	{
		throw "Matrix dimensions do not match";
		return *this;
	}

	for (uint8_t i = 0; i < m._rows; i++)
	{
		for (uint8_t j = 0; j < m._cols; j++)
		{
			this->_data[row + i][col + j] = m._data[i][j];
		}
	}

	return *this;
}

float Matrix::get(uint8_t row, uint8_t col) const
{
	if (row >= this->_rows || col >= this->_cols)
	{
		throw "Matrix index out of bounds";
		return 0;
	}

	return this->_data[row][col];
}

Vector Matrix::getRow(uint8_t row) const
{
	if (row >= this->_rows)
	{
		throw "Matrix index out of bounds";
		return Vector();
	}

	return Vector(this->_data[row], this->_cols);
}

Vector Matrix::getCol(uint8_t col) const
{
	if (col >= this->_cols)
	{
		throw "Matrix index out of bounds";
		return Vector();
	}

	Vector result(this->_rows);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		result[i] = this->_data[i][col];
	}

	return result;
}

Matrix Matrix::getSubMatrix(uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol) const
{
	if (endRow > this->_rows || endCol > this->_cols || startRow > endRow || startCol > endCol)
	{
		throw "Matrix index out of bounds";
		return Matrix();
	}

	Matrix result(endRow - startRow, endCol - startCol);

	for (uint8_t i = startRow; i < endRow; i++)
	{
		for (uint8_t j = startCol; j < endCol; j++)
		{
			result._data[i - startRow][j - startCol] = this->_data[i][j];
		}
	}

	return result;
}

Vector *Matrix::operator[](uint8_t row)
{
	if (row >= this->_rows)
	{
		throw "Matrix index out of bounds";
		return NULL;
	}

	return new Vector(this->_data[row], this->_cols);
}

Vector Matrix::operator[](uint8_t row) const
{
	if (row >= this->_rows)
	{
		throw "Matrix index out of bounds";
		return Vector();
	}

	return Vector(this->_data[row], this->_cols);
}

float &Matrix::operator()(uint8_t row, uint8_t col)
{
	if (row >= this->_rows || col >= this->_cols)
	{
		throw "Matrix index out of bounds";
		return this->_data[0][0];
	}

	return this->_data[row][col];
}

Matrix &Matrix::operator=(const Matrix &m)
{
	if (this->_data != NULL)
	{
		for (uint8_t i = 0; i < this->_rows; i++)
		{
			delete[] this->_data[i];
		}

		delete[] this->_data;
	}

	this->_rows = m._rows;
	this->_cols = m._cols;

	this->_data = new float *[this->_rows];
	for (uint8_t i = 0; i < this->_rows; i++)
	{
		this->_data[i] = new float[this->_cols];

		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] = m._data[i][j];
		}
	}

	return *this;
}

Matrix Matrix::operator+(const Matrix &m) const
{
	if (this->_rows != m._rows || this->_cols != m._cols)
	{
		throw "Matrix dimensions do not match";
		return Matrix();
	}

	Matrix result(this->_rows, this->_cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result._data[i][j] = this->_data[i][j] + m._data[i][j];
		}
	}

	return result;
}

Matrix Matrix::operator-(const Matrix &m) const
{
	if (this->_rows != m._rows || this->_cols != m._cols)
	{
		throw "Matrix dimensions do not match";
		return Matrix();
	}

	Matrix result(this->_rows, this->_cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result._data[i][j] = this->_data[i][j] - m._data[i][j];
		}
	}

	return result;
}

Matrix Matrix::operator*(const Matrix &m) const
{
	if (this->_cols != m._rows)
	{
		throw "Matrix dimensions do not match";
		return Matrix();
	}

	Matrix result(this->_rows, m._cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < m._cols; j++)
		{
			result._data[i][j] = 0;

			for (uint8_t k = 0; k < this->_cols; k++)
			{
				result._data[i][j] += this->_data[i][k] * m._data[k][j];
			}
		}
	}

	return result;
}

Matrix Matrix::operator*(const float &s) const
{
	Matrix result(this->_rows, this->_cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result._data[i][j] = this->_data[i][j] * s;
		}
	}

	return result;
}

Matrix Matrix::operator/(const float &s) const
{
	Matrix result(this->_rows, this->_cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result._data[i][j] = this->_data[i][j] / s;
		}
	}

	return result;
}

void Matrix::operator+=(const Matrix &m)
{
	if (this->_rows != m._rows || this->_cols != m._cols)
	{
		throw "Matrix dimensions do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] += m._data[i][j];
		}
	}
}

void Matrix::operator-=(const Matrix &m)
{
	if (this->_rows != m._rows || this->_cols != m._cols)
	{
		throw "Matrix dimensions do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] -= m._data[i][j];
		}
	}
}

void Matrix::operator*=(const Matrix &m)
{
	if (this->_cols != m._rows)
	{
		throw "Matrix dimensions do not match";
		return;
	}

	Matrix result(this->_rows, m._cols);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < m._cols; j++)
		{
			result._data[i][j] = 0;

			for (uint8_t k = 0; k < this->_cols; k++)
			{
				result._data[i][j] += this->_data[i][k] * m._data[k][j];
			}
		}
	}

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		delete[] this->_data[i];
	}

	delete[] this->_data;

	this->_rows = result._rows;
	this->_cols = result._cols;
	this->_data = result._data;
}

void Matrix::operator*=(const float &s)
{
	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] *= s;
		}
	}
}

void Matrix::operator/=(const float &s)
{
	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			this->_data[i][j] /= s;
		}
	}
}

Vector Matrix::operator*(const Vector &v) const
{
	if (this->_cols != v.size())
	{
		throw "Matrix dimensions do not match";
		return Vector();
	}

	Vector result(this->_rows);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		result[i] = 0;

		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result[i] += this->_data[i][j] * v[j];
		}
	}

	return result;
}

Matrix Matrix::transpose() const
{
	Matrix result(this->_cols, this->_rows);

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		for (uint8_t j = 0; j < this->_cols; j++)
		{
			result._data[j][i] = this->_data[i][j];
		}
	}

	return result;
}

Matrix Matrix::T() const
{
	return this->transpose();
}

Matrix Matrix::inverse() const
{
	if (this->_rows != this->_cols)
	{
		throw "Matrix is not square";
		return Matrix();
	}

	uint8_t size = this->_rows;

	Matrix result(size, size);

	float **data = new float *[size];
	for (uint8_t i = 0; i < size; i++)
	{
		data[i] = new float[size * 2];

		for (uint8_t j = 0; j < size; j++)
		{
			data[i][j] = this->_data[i][j];
			data[i][j + size] = (i == j) ? 1 : 0;
		}
	}

	for (uint8_t i = 0; i < size; i++)
	{
		float pivot = data[i][i];

		if (pivot == 0)
		{
			throw "Matrix is singular";
			return Matrix();
		}

		for (uint8_t j = 0; j < size * 2; j++)
		{
			data[i][j] /= pivot;
		}

		for (uint8_t j = 0; j < size; j++)
		{
			if (j == i)
			{
				continue;
			}

			float factor = data[j][i];

			for (uint8_t k = 0; k < size * 2; k++)
			{
				data[j][k] -= factor * data[i][k];
			}
		}
	}

	for (uint8_t i = 0; i < size; i++)
	{
		for (uint8_t j = 0; j < size; j++)
		{
			result._data[i][j] = data[i][j + size];
		}
	}

	for (uint8_t i = 0; i < size; i++)
	{
		delete[] data[i];
	}

	delete[] data;

	return result;
}

float Matrix::determinant() const
{
	if (this->_rows != this->_cols)
	{
		throw "Matrix is not square";
		return 0;
	}

	uint8_t size = this->_rows;

	float **data = new float *[size];
	for (uint8_t i = 0; i < size; i++)
	{
		data[i] = new float[size];

		for (uint8_t j = 0; j < size; j++)
		{
			data[i][j] = this->_data[i][j];
		}
	}

	float result = 1;

	for (uint8_t i = 0; i < size; i++)
	{
		float pivot = data[i][i];

		if (pivot == 0)
		{
			throw "Matrix is singular";
			return 0;
		}

		result *= pivot;

		for (uint8_t j = 0; j < size; j++)
		{
			data[i][j] /= pivot;
		}

		for (uint8_t j = 0; j < size; j++)
		{
			if (j == i)
			{
				continue;
			}

			float factor = data[j][i];

			for (uint8_t k = 0; k < size; k++)
			{
				data[j][k] -= factor * data[i][k];
			}
		}
	}

	for (uint8_t i = 0; i < size; i++)
	{
		delete[] data[i];
	}

	delete[] data;

	return result;
}

void Matrix::print() const
{
	char buffer[8];
	char *result = new char[this->_rows * this->_cols * 16];

	strcpy(result, "[");

	for (uint8_t i = 0; i < this->_rows; i++)
	{
		strcat(result, "[");

		for (uint8_t j = 0; j < this->_cols; j++)
		{
			sprintf(buffer, "%.2f", this->_data[i][j]);
			strcat(result, buffer);

			if (j < this->_cols - 1)
			{
				strcat(result, ", ");
			}
		}

		strcat(result, "]");

		if (i < this->_rows - 1)
		{
			strcat(result, ", ");
		}
	}

	strcat(result, "]");

	printf("%s\n", result);
	delete[] result;
}

Matrix Matrix::Identity(const uint8_t size)
{
	Matrix result(size, size);

	for (uint8_t i = 0; i < size; i++)
	{
		result._data[i][i] = 1;
	}

	return result;
}

Matrix Matrix::Diagonal(const Vector &v)
{
	Matrix result(v.size(), v.size());

	for (uint8_t i = 0; i < v.size(); i++)
	{
		result._data[i][i] = v[i];
	}

	return result;
}

Matrix Matrix::Diagonal(const std::initializer_list<float> &data)
{
	Matrix result(data.size(), data.size());

	uint8_t i = 0;
	for (const auto &value : data)
	{
		result._data[i][i] = value;
		++i;
	}

	return result;
}

Matrix Matrix::Rotation2D(const float theta)
{
	float s = sin(theta);
	float c = cos(theta);

	Matrix result(2, 2);

	result._data[0][0] = c;
	result._data[0][1] = -s;

	result._data[1][0] = s;
	result._data[1][1] = c;

	return result;
}

Matrix Matrix::Rotation3D(const float theta, const float phi, const float psi)
{
	float s1 = sin(theta);
	float c1 = cos(theta);

	float s2 = sin(phi);
	float c2 = cos(phi);

	float s3 = sin(psi);
	float c3 = cos(psi);

	Matrix result(3, 3);

	result._data[0][0] = c1 * c3;
	result._data[0][1] = c1 * s3;
	result._data[0][2] = -s1;

	result._data[1][0] = -c3 * s3 + s1 * s2 * c3;
	result._data[1][1] = c2 * c3 + s1 * s2 * s3;
	result._data[1][2] = c1 * s2;

	result._data[2][0] = s2 * s3 + s1 * c2 * c3;
	result._data[2][1] = -s2 * c3 + s1 * c2 * s3;
	result._data[2][2] = c1 * c2;

	return result;
}

Matrix Matrix::Rotation3D(const Vector &v)
{
	return Rotation3D(v[0], v[1], v[2]);
}

Matrix Matrix::Rotation3D(const float x, const float y, const float z, const float w)
{
	Matrix result(3, 3);

	result._data[0][0] = 1 - 2 * y * y - 2 * z * z;
	result._data[0][1] = 2 * x * y - 2 * z * w;
	result._data[0][2] = 2 * x * z + 2 * y * w;

	result._data[1][0] = 2 * x * y + 2 * z * w;
	result._data[1][1] = 1 - 2 * x * x - 2 * z * z;
	result._data[1][2] = 2 * y * z - 2 * x * w;

	result._data[2][0] = 2 * x * z - 2 * y * w;
	result._data[2][1] = 2 * y * z + 2 * x * w;
	result._data[2][2] = 1 - 2 * x * x - 2 * y * y;

	return result;
}

Matrix Matrix::Rotation3D(const Quaternion &q)
{
	return Rotation3D(q.x, q.y, q.z, q.w);
}
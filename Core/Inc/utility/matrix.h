#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <math.h>
#include <stdint.h>
#include <initializer_list>
#include <string.h>
#include <stdio.h>
#include "utility/vector.h"
#include "utility/quaternion.h"

class Matrix
{
public:
	Matrix();
	Matrix(const uint8_t size);
	Matrix(const uint8_t rows, const uint8_t cols);
	Matrix(float **data, const uint8_t rows, const uint8_t cols);
	Matrix(const std::initializer_list<std::initializer_list<float>> &data);
	Matrix(const Matrix &m);
	~Matrix();

	uint8_t rows() const;
	uint8_t cols() const;

	Matrix set(uint8_t row, uint8_t col, float value);
	Matrix setRow(uint8_t row, const Vector &v);
	Matrix setCol(uint8_t col, const Vector &v);
	Matrix setSubMatrix(uint8_t startRow, uint8_t startCol, const Matrix &m);

	float get(uint8_t row, uint8_t col) const;
	Vector getRow(uint8_t row) const;
	Vector getCol(uint8_t col) const;
	Matrix getSubMatrix(uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol) const;

	Vector *operator[](uint8_t row);
	Vector operator[](uint8_t row) const;

	float &operator()(uint8_t row, uint8_t col);

	Matrix &operator=(const Matrix &m);

	Matrix operator+(const Matrix &m) const;
	Matrix operator-(const Matrix &m) const;
	Matrix operator*(const Matrix &m) const;
	Matrix operator*(const float &s) const;
	Matrix operator/(const float &s) const;

	void operator+=(const Matrix &m);
	void operator-=(const Matrix &m);
	void operator*=(const Matrix &m);
	void operator*=(const float &s);
	void operator/=(const float &s);

	Vector operator*(const Vector &v) const;

	Matrix transpose() const;
	Matrix T() const;
	Matrix inverse() const;
	float determinant() const;

	void print() const;

	static Matrix Identity(const uint8_t size);
	static Matrix Diagonal(const Vector &v);
	static Matrix Diagonal(const std::initializer_list<float> &data);

	static Matrix Rotation2D(const float theta);
	static Matrix Rotation3D(const float theta, const float phi, const float psi);
	static Matrix Rotation3D(const Vector &v);
	static Matrix Rotation3D(const float x, const float y, const float z, const float w);
	static Matrix Rotation3D(const Quaternion &q);

private:
	uint8_t _rows;
	uint8_t _cols;
	float **_data;
};

#endif // __MATRIX_H__
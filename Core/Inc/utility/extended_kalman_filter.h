#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include "utility/vector.h"
#include "utility/matrix.h"

class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter();
	// Asynchronous communication
	ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), const Matrix &Q);
	// Synchronous communication
	ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), const Matrix &Q, Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R);
	~ExtendedKalmanFilter();

	void initialize(const Vector &x, const Matrix &P);

	// Predict the state estimate
	void predict(const Vector &u);

	// Synchronous update
	void update(const Vector &z);

	// Asynchronous update, updates measurement function and Jacobian then updates the state estimate
	void asyncUpdate(const Vector &z, Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R);

	void setMeasurement(Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R);

	Vector getState() const;

private:
	uint8_t _state_size;	   // State size
	uint8_t _measurement_size; // Measurement size

	Vector (*_f)(const Vector &x, const Vector &u); // State transition function
	Matrix (*_F)(const Vector &x, const Vector &u); // State transition Jacobian
	Matrix _Q;										// Process noise covariance

	Vector (*_h)(const Vector &x); // Measurement function
	Matrix (*_H)(const Vector &x); // Measurement Jacobian
	Matrix _R;					   // Measurement noise covariance

	Vector _x; // State estimate
	Matrix _P; // Estimate covariance
};

#endif // __EXTENDED_KALMAN_FILTER_H__
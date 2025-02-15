#include "extended_kalman_filter.h"

// ExtendedKalmanFilter class implementation

ExtendedKalmanFilter::ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), const Matrix &Q)
{
	this->_f = f;
	this->_F = F;
	this->_Q = Q;

	this->_state_size = Q.rows();
	this->_measurement_size = 0;

	this->_x = Vector(this->_state_size);
	this->_P = Matrix::Identity(this->_state_size);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), const Matrix &Q, Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R)
{
	this->_f = f;
	this->_F = F;
	this->_Q = Q;

	this->_h = h;
	this->_H = H;
	this->_R = R;

	this->_state_size = Q.rows();
	this->_measurement_size = R.rows();

	this->_x = Vector(this->_state_size);
	this->_P = Matrix::Identity(this->_state_size);
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
}

void ExtendedKalmanFilter::initialize(const Vector &x, const Matrix &P)
{
	this->_state_size = x.size();

	this->_x = x;
	this->_P = P;
}

void ExtendedKalmanFilter::predict(const Vector &u)
{
	this->_Fm = this->_F(this->_x, u); // Jacobian of state transition function
	
	// Predict the state estimate
	this->_x = this->_f(this->_x, u);
	this->_P = _Fm * this->_P * _Fm.transpose() + this->_Q;
}

void ExtendedKalmanFilter::update(const Vector &z)
{
	this->_Hm = this->_H(this->_x); // Jacobian of measurement function

	// Calculate the Kalman gain
	this->_K = this->_P * _Hm.transpose() * (_Hm * this->_P * _Hm.transpose() + this->_R).inverse();

	// Update the state estimate
	this->_x += _K * (z - this->_h(this->_x));
	this->_P = (Matrix::Identity(this->_state_size) - _K * _Hm) * this->_P;
}

void ExtendedKalmanFilter::asyncUpdate(const Vector &z, Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R)
{
	this->setMeasurement(h, H, R);
	this->update(z);
}

void ExtendedKalmanFilter::setMeasurement(Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &R)
{
	this->_h = h;
	this->_H = H;
	this->_R = R;

	this->_measurement_size = R.rows();
}

Vector ExtendedKalmanFilter::getState() const
{
	return this->_x;
}
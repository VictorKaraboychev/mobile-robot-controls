#include "extended_kalman_filter.h"

// ExtendedKalmanFilter class implementation

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
}

ExtendedKalmanFilter::ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), const Matrix &Q, const Matrix &R)
{
	this->_f = f;
	this->_F = F;

	this->_h = h;
	this->_H = H;

	this->_Q = Q;
	this->_R = R;

	this->_state_size = Q.rows();
	this->_measurement_size = R.rows();

	this->_x = Vector(this->_state_size);
	this->_P = Matrix::identity(this->_state_size);
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
	Vector f = this->_f(this->_x, u); // State transition function
	Matrix F = this->_F(this->_x, u); // Jacobian of state transition function

	// Predict the state estimate
	this->_x += f;
	this->_P += (F * this->_P + this->_P * F.transpose() + this->_Q);
}

void ExtendedKalmanFilter::update(const Vector &z, int8_t i_start, int8_t i_end)
{
	if (i_end < 0)
	{
		i_end = this->_measurement_size;
	}

	Vector h = this->_h(this->_x).getSubVector(i_start, i_end);							  // Measurement function
	Matrix H = this->_H(this->_x).getSubMatrix(i_start, i_end, 0, this->_state_size - 1); // Jacobian of measurement function
	Matrix R = this->_R.getSubMatrix(i_start, i_end, i_start, i_end);					  // Measurement noise covariance

	// Calculate the Kalman gain
	Matrix K = this->_P * H.transpose() * (H * this->_P * H.transpose() + R).inverse();

	// Update the state estimate
	this->_x += K * (z - h);
	this->_P = (Matrix::identity(this->_P.rows()) - K * H) * this->_P;
}

Vector ExtendedKalmanFilter::getState() const
{
	return this->_x;
}
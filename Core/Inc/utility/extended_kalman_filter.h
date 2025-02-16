#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <functional>

// Template parameter StateDim is the fixed state dimension.
template <typename DataType = float, int StateDim = Eigen::Dynamic, int ControlDim = Eigen::Dynamic, int MeasurementDim = Eigen::Dynamic>
class ExtendedKalmanFilter
{
public:
	// Fixed-size types for the state.
	using StateVector = Eigen::Matrix<DataType, StateDim, 1>;
	using StateMatrix = Eigen::Matrix<DataType, StateDim, StateDim>;

	// Process types are fixed size because the process model is fixed.
	using ProcessVector = Eigen::Matrix<DataType, StateDim, 1>;
	using ProcessJacobian = Eigen::Matrix<DataType, StateDim, StateDim>;
	using ProcessCovariance = Eigen::Matrix<DataType, StateDim, StateDim>;

	// Control vector is dynamic sized (you can change this if control dimensions are fixed).
	using ControlVector = Eigen::Matrix<DataType, ControlDim, 1>;

	// Measurement types are dynamic because different sensors can have different measurement dimensions.
	using MeasurementVector = Eigen::Matrix<DataType, MeasurementDim, 1>;
	using MeasurementJacobian = Eigen::Matrix<DataType, MeasurementDim, StateDim>;
	using MeasurementCovariance = Eigen::Matrix<DataType, MeasurementDim, MeasurementDim>;

	// Process model function types.
	// f: state update given current state and control input.
	using ProcessFunc = std::function<ProcessVector(const StateVector &, const ControlVector &)>;
	// F: Jacobian of the process model.
	using ProcessJacobianFunc = std::function<ProcessJacobian(const StateVector &, const ControlVector &)>;

	// Measurement model function types.
	// h: measurement prediction from the current state.
	using MeasurementFunc = std::function<MeasurementVector(const StateVector &)>;
	// H: Jacobian of the measurement model.
	using MeasurementJacobianFunc = std::function<MeasurementJacobian(const StateVector &)>;

	// Constructor for synchronous communication.
	ExtendedKalmanFilter(ProcessFunc f, ProcessJacobianFunc F, const ProcessCovariance &Q);
	ExtendedKalmanFilter(ProcessFunc f, ProcessJacobianFunc F, const ProcessCovariance &Q, MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R);

	~ExtendedKalmanFilter();

	// Initialize the filter with a state vector and covariance matrix.
	void initialize(const StateVector &x, const StateMatrix &P);

	// Predict step with control input.
	void predict(const ControlVector &u);

	// Update step using measurement z.
	void update(const MeasurementVector &z);

	// Asynchronous update: set measurement functions and noise covariance then update.
	void asyncUpdate(const MeasurementVector &z, MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R);

	// Change the measurement functions and covariance.
	void setMeasurement(MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R);

	// Get the current state estimate.
	StateVector getState() const;

private:
	// Process model.
	ProcessFunc _f;
	ProcessJacobianFunc _F;
	ProcessCovariance _Q; // Process noise covariance.

	// Measurement model.
	MeasurementFunc _h;
	MeasurementJacobianFunc _H;
	MeasurementCovariance _R; // Measurement noise covariance.

	// Filter estimates.
	StateVector x; // State estimate.
	StateMatrix P; // Covariance estimate.
	StateMatrix I; // Precomputed identity matrix for the state (StateDim x StateDim).
};

// Implementation of the templated ExtendedKalmanFilter class.

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::ExtendedKalmanFilter(ProcessFunc f, ProcessJacobianFunc F, const ProcessCovariance &Q)
	: _f(f), _F(F), _Q(Q)
{
	this->x = StateVector::Zero();
	this->P = StateMatrix::Identity();
	this->I = StateMatrix::Identity();
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::ExtendedKalmanFilter(ProcessFunc f, ProcessJacobianFunc F, const ProcessCovariance &Q, MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R)
	: _f(f), _F(F), _Q(Q), _h(h), _H(H), _R(R)
{
	this->x = StateVector::Zero();
	this->P = StateMatrix::Identity();
	this->I = StateMatrix::Identity();
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::~ExtendedKalmanFilter()
{
	// No dynamic allocation to free here.
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
void ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::initialize(const StateVector &x, const StateMatrix &P)
{
	this->x = x;
	this->P = P;
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
void ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::predict(const ControlVector &u)
{
	// Evaluate process model and its Jacobian.
	StateVector f_val = _f(this->x, u);
	StateMatrix F_val = _F(this->x, u);

	// Propagate state and covariance.
	this->x = f_val;
	this->P = F_val * this->P * F_val.transpose() + _Q;
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
void ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::update(const MeasurementVector &z)
{
	// Evaluate measurement model and its Jacobian.
	MeasurementVector h_val = _h(this->x);
	MeasurementJacobian H_val = _H(this->x);

	// Compute the innovation covariance: S = H * P * H' + R.
	MeasurementCovariance S = H_val * this->P * H_val.transpose() + _R;

	// Compute the Kalman gain: K = P * H' * S⁻¹.
	// K has dimensions (StateDim x measurement dimension).
	Eigen::Matrix<DataType, StateDim, MeasurementDim> K = this->P * H_val.transpose() * S.inverse();

	// Update the state estimate.
	this->x += K * (z - h_val);

	// Update the covariance estimate.
	this->P = (this->I - K * H_val) * this->P;
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
void ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::asyncUpdate(const MeasurementVector &z, MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R)
{
	setMeasurement(h, H, R);
	update(z);
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
void ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::setMeasurement(MeasurementFunc h, MeasurementJacobianFunc H, const MeasurementCovariance &R)
{
	_h = h;
	_H = H;
	_R = R;
}

template <typename DataType, int StateDim, int ControlDim, int MeasurementDim>
typename ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::StateVector ExtendedKalmanFilter<DataType, StateDim, ControlDim, MeasurementDim>::getState() const
{
	return this->x;
}

#endif // EXTENDED_KALMAN_FILTER_H

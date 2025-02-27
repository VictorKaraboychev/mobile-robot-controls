#include "magnetometer.h"

extern osMutexId_t spi1MutexHandle;

LIS2MDL_Object_t lis2mdl;
MagnetometerData magnetometer_data;

// Structure to hold calibration parameters.
struct MagnetometerCalibration
{
	Eigen::Vector3f hardIronBias;	// Hard-iron offset.
	Eigen::Matrix3f softIronMatrix; // Soft-iron correction matrix.
};

// /**
//  * @brief Calibrates a 3D magnetometer by fitting an ellipsoid to the raw data.
//  *
//  * @param measurements A vector of raw 3D magnetometer measurements (e.g., from LIS2MDL).
//  * @return MagCalibrationData containing the hard iron bias and soft iron correction matrix.
//  */
// // This function implements the "Direct Least Square Fitting of Ellipsoids" approach.
// // It takes raw magnetometer measurements (as a vector of Eigen::Vector3f),
// // fits an ellipsoid to the data, and computes the hard-iron bias (ellipsoid center)
// // and the soft-iron correction matrix (to map the ellipsoid to a unit sphere).
// MagnetometerCalibration CalibrateMagnetometer(const std::vector<Eigen::Vector3f> &rawData)
// {
// 	int N = rawData.size();
// 	if (N < 10)
// 	{
// 		printf("Not enough data points for calibration\n");
// 		throw std::runtime_error("Not enough data points for calibration");
// 	}

// 	// Build design matrix D (size: N x 10).
// 	// Each row corresponds to: [x^2, y^2, z^2, x*y, x*z, y*z, x, y, z, 1].
// 	Eigen::Matrix<float, Eigen::Dynamic, 10> D(N, 10);
// 	for (int i = 0; i < N; i++)
// 	{
// 		float x = rawData[i](0);
// 		float y = rawData[i](1);
// 		float z = rawData[i](2);
// 		D(i, 0) = x * x;
// 		D(i, 1) = y * y;
// 		D(i, 2) = z * z;
// 		D(i, 3) = x * y;
// 		D(i, 4) = x * z;
// 		D(i, 5) = y * z;
// 		D(i, 6) = x;
// 		D(i, 7) = y;
// 		D(i, 8) = z;
// 		D(i, 9) = 1.0;
// 	}

// 	// Compute the scatter matrix: S = D^T * D (10 x 10).
// 	Eigen::Matrix<float, 10, 10> S = D.transpose() * D;

// 	// Build the 10x10 constraint matrix C.
// 	// Only the first 6 (quadratic) terms are constrained:
// 	//   C(0,0)=1, C(1,1)=1, C(2,2)=1,
// 	//   C(3,3)=2, C(4,4)=2, C(5,5)=2.
// 	Eigen::Matrix<float, 10, 10> C = Eigen::Matrix<float, 10, 10>::Identity() * 1e-10;
// 	C(0, 0) = 1;
// 	C(1, 1) = 1;
// 	C(2, 2) = 1;
// 	C(3, 3) = 2;
// 	C(4, 4) = 2;
// 	C(5, 5) = 2;
// 	// The remaining rows/cols remain zero.

// 	// Solve the generalized eigenvalue problem: S * v = lambda * C * v.
// 	// We use Eigen's GeneralizedSelfAdjointEigenSolver.
// 	Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXf> ges(S, C);
// 	if (ges.info() != Eigen::Success)
// 	{
// 		printf("Generalized eigenvalue decomposition failed\n");
// 		throw std::runtime_error("Generalized eigenvalue decomposition failed");
// 	}

// 	// Get the eigenvalues and eigenvectors.
// 	Eigen::Vector<float, 10> eigenvalues = ges.eigenvalues();
// 	Eigen::Matrix<float, 10, 10> eigenvectors = ges.eigenvectors(); // Each column is an eigenvector.

// 	// Select the eigenvector corresponding to the smallest positive eigenvalue.
// 	float minPosEig = std::numeric_limits<float>::max();
// 	int bestIndex = -1;
// 	for (int i = 0; i < eigenvalues.size(); i++)
// 	{
// 		float val = eigenvalues(i);
// 		if (val > 1e-8 && val < minPosEig)
// 		{
// 			minPosEig = val;
// 			bestIndex = i;
// 		}
// 	}
// 	if (bestIndex == -1)
// 	{
// 		printf("No positive eigenvalue found in ellipsoid fit\n");
// 		throw std::runtime_error("No positive eigenvalue found in ellipsoid fit");
// 	}

// 	// The 10x1 solution vector v.
// 	Eigen::Vector<float, 10> v = eigenvectors.col(bestIndex);

// 	// Assume v is the eigenvector we obtained, with v(9) corresponding to j.
// 	if (std::abs(v(9)) > 1e-8)
// 	{
// 		// Normalize so that v(9) becomes -1.
// 		v = -v / v(9);
// 	}
// 	else
// 	{
// 		printf("v(9) is too close to zero, cannot normalize\n");
// 		throw std::runtime_error("v(9) is too close to zero, cannot normalize");
// 	}

// 	// v = [a, b, c, d, e, f, g, h, i, j]^T corresponds to the ellipsoid:
// 	//   a*x^2 + b*y^2 + c*z^2 + d*x*y + e*x*z + f*y*z + g*x + h*y + i*z + j = 0.

// 	// Construct the symmetric matrix A for the quadratic form (3x3):
// 	//   A = [ a       d/2     e/2;
// 	//         d/2     b       f/2;
// 	//         e/2     f/2     c   ]
// 	Eigen::Matrix3f A_mat;
// 	A_mat(0, 0) = v(0);
// 	A_mat(1, 1) = v(1);
// 	A_mat(2, 2) = v(2);
// 	A_mat(0, 1) = v(3) / 2.0;
// 	A_mat(1, 0) = v(3) / 2.0;
// 	A_mat(0, 2) = v(4) / 2.0;
// 	A_mat(2, 0) = v(4) / 2.0;
// 	A_mat(1, 2) = v(5) / 2.0;
// 	A_mat(2, 1) = v(5) / 2.0;

// 	// Linear part (3x1 vector):
// 	Eigen::Vector3f b_vec;
// 	b_vec << v(6), v(7), v(8);

// 	float j_const = v(9);

// 	// Compute the ellipsoid center (hard-iron bias):
// 	//   center = -0.5 * inv(A_mat) * b_vec.
// 	Eigen::Vector3f center = -0.5 * A_mat.ldlt().solve(b_vec);

// 	// Compute the translated constant:
// 	// Compute T = center^T * A_mat * center - j_const.
// 	float T = center.transpose() * A_mat * center - j_const;
// 	if (T <= 0)
// 	{
// 		// Flip the sign of v to correct the sign of T.
// 		v = -v;
// 		// Update the components accordingly:
// 		// Recompute A_mat, b_vec, and j_const from the new v.
// 		A_mat(0, 0) = v(0);
// 		A_mat(1, 1) = v(1);
// 		A_mat(2, 2) = v(2);
// 		A_mat(0, 1) = v(3) / 2.0;
// 		A_mat(1, 0) = v(3) / 2.0;
// 		A_mat(0, 2) = v(4) / 2.0;
// 		A_mat(2, 0) = v(4) / 2.0;
// 		A_mat(1, 2) = v(5) / 2.0;
// 		A_mat(2, 1) = v(5) / 2.0;
// 		b_vec << v(6), v(7), v(8);
// 		j_const = v(9);

// 		// The center remains the same by construction, but recalc T:
// 		T = center.transpose() * A_mat * center - j_const;
// 		if (T <= 0)
// 		{
// 			printf("Invalid ellipsoid fit: computed T is non-positive even after sign correction\n");
// 			throw std::runtime_error("Invalid ellipsoid fit: computed T is non-positive even after sign correction");
// 		}
// 	}

// 	// Normalize A_mat to get A_norm such that in shifted coordinates:
// 	//   (x - center)^T * A_norm * (x - center) = 1.
// 	Eigen::Matrix3f A_norm = A_mat / T;

// 	// Decompose A_norm to extract ellipsoid axes and orientation.
// 	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigSolver(A_norm);
// 	if (eigSolver.info() != Eigen::Success)
// 	{
// 		printf("Eigen decomposition of A_norm failed\n");
// 		throw std::runtime_error("Eigen decomposition of A_norm failed");
// 	}
// 	Eigen::Vector3f lambda = eigSolver.eigenvalues();
// 	Eigen::Matrix3f V = eigSolver.eigenvectors(); // Columns are the principal directions.

// 	// Compute semi-axis lengths: axes[i] = 1/sqrt(lambda_i).
// 	Eigen::Vector3f axes;
// 	float tol = 1e-8; // Tolerance (adjust as needed)
// 	for (int i = 0; i < 3; i++)
// 	{
// 		if (lambda(i) < tol)
// 		{
// 			printf("Warning: eigenvalue %.10f is below tolerance, setting to %.10f\n", lambda(i), tol);
// 			lambda(i) = tol;
// 		}
// 		axes(i) = 1.0 / std::sqrt(lambda(i));
// 	}

// 	// For calibration, we want to map the ellipsoid to a unit sphere.
// 	// The correction applied to a measurement x is:
// 	//   x_corrected = softIronMatrix * (x - center)
// 	// We choose:
// 	//   softIronMatrix = V * diag(1./axes) * V^T.
// 	// Since 1./axes = sqrt(lambda), we can also write:
// 	//   softIronMatrix = V * diag(sqrt(lambda)) * V^T.
// 	Eigen::Matrix3f D_sqrt = Eigen::Matrix3f::Zero();
// 	for (int i = 0; i < 3; i++)
// 	{
// 		D_sqrt(i, i) = std::sqrt(lambda(i));
// 	}
// 	Eigen::Matrix3f softIronMatrix = V * D_sqrt * V.transpose();

// 	// Return calibration parameters.
// 	MagnetometerCalibration calibration;
// 	calibration.hardIronBias = center;
// 	calibration.softIronMatrix = softIronMatrix;
// 	return calibration;
// }

// Example function to print points in MATLAB format
void printPointsForMatlab(const std::vector<Eigen::Vector3f> &points)
{
	// Print the start of the MATLAB matrix definition
	printf("data = [\n");
	for (size_t i = 0; i < points.size(); i++)
	{
		// Print each point on its own line, separated by spaces and ended with a semicolon
		printf("  %.4f %.4f %.4f;\n", points[i](0), points[i](1), points[i](2));
		osDelay(1); // Delay to prevent the console from overflowing
	}
	// Close the MATLAB matrix
	printf("];\n");
}

int32_t Write_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

void StartMagTask(void *argument)
{
	lis2mdl.Ctx.handle = &hspi1;
	lis2mdl.Ctx.write_reg = Write_LIS2MDL;
	lis2mdl.Ctx.read_reg = Read_LIS2MDL;
	lis2mdl.Ctx.mdelay = HAL_Delay;

	lis2mdl.IO.BusType = LIS2MDL_SPI_4WIRES_BUS;

	magnetometer_data.active = false;

	uint8_t id = 0, rst = 0;

	// Reset the magnetometer
	lis2mdl_reset_set(&lis2mdl.Ctx, PROPERTY_ENABLE);
	do
	{
		lis2mdl_reset_get(&lis2mdl.Ctx, &rst);
	} while (rst);

	// Set the SPI mode to 4-wire
	lis2mdl_spi_mode_set(&lis2mdl.Ctx, LIS2MDL_SPI_4_WIRE);

	// Read the magnetometer ID
	do
	{
		LIS2MDL_ReadID(&lis2mdl, &id);
		printf("Magnetometer ID: 0x%02X\n", id);
		osDelay(100);
	} while (id != LIS2MDL_ID);

	// Initialize the magnetometer
	LIS2MDL_Init(&lis2mdl);
	lis2mdl_spi_mode_set(&lis2mdl.Ctx, LIS2MDL_SPI_4_WIRE);

	// Magnetometer configuration
	LIS2MDL_MAG_SetOutputDataRate(&lis2mdl, 100.0f);
	LIS2MDL_MAG_SetFullScale(&lis2mdl, 50);
	LIS2MDL_MAG_Set_Power_Mode(&lis2mdl, LIS2MDL_HIGH_RESOLUTION);
	LIS2MDL_MAG_Enable(&lis2mdl);

	// Wait for the magnetometer to start
	osDelay(100);

	// Magnetometer scale factor
	float mag_scale = 1.5f / 1000.0f;

	LIS2MDL_Axes_t raw;
	float roll, pitch, yaw;
	bool status = false;

	// Calibrate the magnetometer
	// uint16_t samples = 256;
	// uint16_t sample_time = 30; // seconds

	// std::vector<Eigen::Vector3f> rawMeasurements;
	// rawMeasurements.reserve(samples);

	// // Set the calibration flag
	// magnetometer_data.is_calibrating = true;

	// for (uint16_t i = 0; i < samples; i++)
	// {
	// 	LIS2MDL_MAG_GetAxes(&lis2mdl, &raw);
	// 	rawMeasurements.push_back(Eigen::Vector3f((float)raw.x, (float)raw.y, (float)raw.z) * mag_scale);

	// 	osDelay(sample_time * 1000 / samples);
	// }

	// Print the raw measurements for MATLAB
	// printPointsForMatlab(rawMeasurements);

	MagnetometerCalibration calibration;
	calibration.hardIronBias = Eigen::Vector3f{-822.5716f, -632.6323f, -266.8716f};
	calibration.softIronMatrix = Eigen::Matrix3f{
		{0.001829009496659f, 2.190811212463005e-05f, 3.800733851654905e-05f},
		{2.190811212463021e-05f, 0.001887173408513f, 7.472021321406437e-05f},
		{3.800733851654902e-05f, 7.472021321406431e-05f, 0.002301448479405f}};

	osDelay(10);

	// printf("Calibrating magnetometer...\n");

	// try
	// {
	// 	// MagnetometerCalibration calibration = CalibrateMagnetometer(rawMeasurements);

	// 	// Print the calibration parameters as a MATLAB matrix
	// 	osDelay(1);
	// 	printf("hardIron = [\n");
	// 	osDelay(1);
	// 	printf("  %.4f %.4f %.4f;\n", calibration.hardIronBias(0), calibration.hardIronBias(1), calibration.hardIronBias(2));
	// 	osDelay(1);
	// 	printf("];\n");
	// 	osDelay(1);

	// 	printf("softIron = [\n");
	// 	osDelay(1);
	// 	printf("  %.4f %.4f %.4f;\n", calibration.softIronMatrix(0, 0), calibration.softIronMatrix(0, 1), calibration.softIronMatrix(0, 2));
	// 	osDelay(1);
	// 	printf("  %.4f %.4f %.4f;\n", calibration.softIronMatrix(1, 0), calibration.softIronMatrix(1, 1), calibration.softIronMatrix(1, 2));
	// 	osDelay(1);
	// 	printf("  %.4f %.4f %.4f;\n", calibration.softIronMatrix(2, 0), calibration.softIronMatrix(2, 1), calibration.softIronMatrix(2, 2));
	// 	osDelay(1);
	// 	printf("];\n");
	// }
	// catch (const std::exception &ex)
	// {
	// 	printf("Error during magnetometer calibration: %s\n", ex.what());
	// }

	// magnetometer_data.is_calibrating = false;

	// // Free the memory used for calibration
	// rawMeasurements.clear();

	while (1)
	{
		// Read the magnetometer data
		status = (LIS2MDL_MAG_GetAxes(&lis2mdl, &raw) == LIS2MDL_OK);

		// Check if the IMU is active
		if (!status || id != LIS2MDL_ID)
		{
			// Update the magnetometer data structure
			magnetometer_data.active = false;
			magnetometer_data.data_ready = false;

			printf("Magnetometer not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		Eigen::Vector3f magnetic_field = calibration.softIronMatrix * (Eigen::Vector3f{(float)raw.x, (float)raw.y, (float)raw.z} - calibration.hardIronBias);

		// Print magnetic field
		// printf("Magnetic Field: %.2f %.2f %.2f\n", magnetic_field[0], magnetic_field[1], magnetic_field[2]);

		// Compute the magnetic orientation
		roll = atan2(magnetic_field[1], magnetic_field[2]);
		pitch = atan2(-magnetic_field[0], hypot(magnetic_field[1], magnetic_field[2]));
		yaw = atan2(magnetic_field[2], magnetic_field[0]);

		magnetometer_data.magnetic_orientation = Eigen::Vector3f{roll, pitch, yaw};

		// Update the magnetometer data structure
		magnetometer_data.active = true;
		magnetometer_data.data_ready = true;

		// printf("ID: 0x%02X Roll: %.1f Pitch: %.1f Yaw: %.1f\n", id, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

		osDelay(10); // 100 Hz
	}
}

// Measurement function
EKF::MeasurementVector h_magnetometer(const EKF::StateVector &x)
{
	EKF::MeasurementVector v(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE);

	v << x[5]; // ψ (yaw)

	return v;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_magnetometer(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 5) = 1; // ∂ψ/∂ψ

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_magnetometer = Eigen::DiagonalMatrix<float, KALMAN_MAGNETOMETER_MEASUREMENT_SIZE>{{
	1.0e-4f // ψ (yaw)
}};

EKF::MeasurementVector magnetometerMeasurement(const EKF::StateVector &x)
{
	float yaw = magnetometer_data.magnetic_orientation[2];

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE);
	z << yaw; // ψ (yaw)

	// Update the state estimate
	return z;
}

bool magnetometerDataReady()
{
	return magnetometer_data.data_ready;
}

Sensor magnetometer = {
	.h = h_magnetometer,
	.H = H_magnetometer,
	.R = R_magnetometer,
	.z = magnetometerMeasurement,
	.ready = magnetometerDataReady};
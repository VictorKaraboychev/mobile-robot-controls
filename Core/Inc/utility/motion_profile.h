#ifndef __MOTION_PROFILE_H__
#define __MOTION_PROFILE_H__

#include <cmath>

class TrapezoidalMotionProfile
{
public:
	TrapezoidalMotionProfile()
		: maxSpeed(0.0), maxAccel(0.0),
		  startPos(0.0), startVel(0.0),
		  targetPos(0.0), targetVel(0.0),
		  currentPos(0.0), currentVel(0.0), currentAccel(0.0),
		  currentTime(0.0), profileComputed(false)
	{
	}

	// Constructor takes the maximum speed and acceleration (magnitudes).
	TrapezoidalMotionProfile(double maxSpeed, double maxAccel)
		: maxSpeed(maxSpeed), maxAccel(maxAccel),
		  startPos(0.0), startVel(0.0),
		  targetPos(0.0), targetVel(0.0),
		  currentPos(0.0), currentVel(0.0), currentAccel(0.0),
		  currentTime(0.0), profileComputed(false)
	{
	}

	// Initialize the profile with an initial position and (optionally) velocity.
	void init(double pos, double vel = 0.0)
	{
		startPos = pos;
		startVel = vel;
		currentPos = pos;
		currentVel = vel;
		currentAccel = 0.0;
		currentTime = 0.0;
		profileComputed = false;
	}

	// Sets the target position and (optionally) target velocity.
	// This function computes the parameters for the profile.
	void setTarget(double targetPosition, double targetVelocity = 0.0)
	{
		targetPos = targetPosition;
		targetVel = targetVelocity;
		computeProfile();
		currentTime = 0.0;
	}

	// Advances the profile by dt seconds.
	void update(double dt)
	{
		if (!profileComputed)
			return;
		currentTime += dt;

		// If we've reached (or passed) the total planned time,
		// clamp the state to the target values.
		if (currentTime >= totalTime)
		{
			currentPos = targetPos;
			currentVel = targetVel;
			currentAccel = 0.0;
		}
		// Phase 1: Acceleration phase
		else if (currentTime < t1)
		{
			double t = currentTime;
			currentAccel = maxAccel * direction;
			currentVel = startVel + currentAccel * t;
			currentPos = startPos + startVel * t + 0.5 * currentAccel * t * t;
		}
		// Phase 2: Constant velocity phase (if any)
		else if (currentTime < t1 + t2)
		{
			double t = currentTime - t1;
			currentAccel = 0.0;
			currentVel = v_peak * direction;
			// Position at beginning of phase 2 is startPos plus the distance covered during phase 1.
			currentPos = startPos + d_acc * direction + currentVel * t;
		}
		// Phase 3: Deceleration phase
		else
		{
			double t = currentTime - t1 - t2;
			currentAccel = -maxAccel * direction;
			// Velocity decreases from v_peak toward target velocity.
			currentVel = v_peak * direction + currentAccel * t;
			// Position: start plus the distances from acceleration, constant, and deceleration phases.
			currentPos = startPos + (d_acc + d_const) * direction +
						 v_peak * t * direction + 0.5 * currentAccel * t * t;
		}
	}

	// Getters for the current state.
	double getPosition() const { return currentPos; }
	double getVelocity() const { return currentVel; }
	double getAcceleration() const { return currentAccel; }

	// Returns true if the motion profile has finished.
	bool isFinished() const { return currentTime >= totalTime; }

	// Optionally, a getter for elapsed time.
	double getTime() const { return currentTime; }

private:
	// Limits (positive magnitudes)
	double maxSpeed;
	double maxAccel;

	// Initial state
	double startPos;
	double startVel;

	// Target state
	double targetPos;
	double targetVel;

	// Current state
	double currentPos;
	double currentVel;
	double currentAccel;
	double currentTime;

	// Profile phase durations and distances:
	double t1, t2, t3, totalTime; // times for accel, constant, decel phases
	double d_acc, d_const, d_dec; // distances for accel, constant, decel phases
	double v_peak;				  // the maximum velocity reached (may be < maxSpeed for a triangular profile)

	// Direction of motion (+1 or -1)
	double direction;

	bool profileComputed;

	// Computes the motion profile parameters.
	void computeProfile()
	{
		// Compute overall displacement and determine the motion direction.
		double d = targetPos - startPos;
		direction = (d >= 0.0) ? 1.0 : -1.0;
		double d_abs = std::fabs(d);

		// Work with magnitudes for velocity.
		double v0 = std::fabs(startVel);
		double vt = std::fabs(targetVel);

		// Compute the candidate peak velocity for a triangular profile.
		// Derived from: d = (v_peak^2 - v0^2)/(2*a) + (v_peak^2 - vt^2)/(2*a)
		double v_peak_candidate = std::sqrt(maxAccel * d_abs + 0.5 * (v0 * v0 + vt * vt));

		// Decide whether we have a trapezoidal (with a constant velocity phase) or triangular profile.
		if (v_peak_candidate > maxSpeed)
		{
			// Trapezoidal profile: we can reach maxSpeed.
			v_peak = maxSpeed;
			t1 = (v_peak - v0) / maxAccel;
			t3 = (v_peak - vt) / maxAccel;
			d_acc = v0 * t1 + 0.5 * maxAccel * t1 * t1;
			d_dec = v_peak * t3 - 0.5 * maxAccel * t3 * t3;
			d_const = d_abs - (d_acc + d_dec);
			t2 = d_const / v_peak;
		}
		else
		{
			// Triangular profile: peak velocity is lower than maxSpeed.
			v_peak = v_peak_candidate;
			t1 = (v_peak - v0) / maxAccel;
			t3 = (v_peak - vt) / maxAccel;
			t2 = 0.0;
			d_acc = v0 * t1 + 0.5 * maxAccel * t1 * t1;
			d_dec = v_peak * t3 - 0.5 * maxAccel * t3 * t3;
			d_const = 0.0;
		}
		totalTime = t1 + t2 + t3;
		profileComputed = true;
	}
};

#endif // __MOTION_PROFILE_H__
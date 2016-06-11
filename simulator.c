//------------------------------------------------------------------------------
//	SIMULATOR.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <math.h>

//------------------------------------------------------------------------------
//	2D QUADCOPTER CONSTANTS
//------------------------------------------------------------------------------
#define M		0.3			// Quadcopter mass in Kg
#define L		0.15		// Quadcopter length in m
#define IZ		0.0005625	// Inertial moment (m * l ^ 2) /12
#define G		9.81		// Gravitational acceleration in m/s^2

//------------------------------------------------------------------------------
//	TRAJECTORY GENERATION CONSTANTS
//------------------------------------------------------------------------------
#define	TRAJ_9	-70			// 9-th order coefficient of trajectory polynomial
#define TRAJ_8	315			// 8-th order coefficient of trajectory polynomial
#define TRAJ_7	-540		// 7-th order coefficient of trajectory polynomial
#define TRAJ_6	420			// 6-th order coefficient of trajectory polynomial
#define TRAJ_5	-126		// 5-th order coefficient of trajectory polynomial

//------------------------------------------------------------------------------
//	Function trajectory
// 	returns the point along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory(float t, float tf, float p0, float pf)
{
float	p;
		p = p0 +\
			pow(t, 9) * (TRAJ_9 * p0 / pow(tf, 9) - TRAJ_9 * pf / pow(tf, 9)) +\
			pow(t, 8) * (TRAJ_8 * p0 / pow(tf, 8) - TRAJ_8 * pf / pow(tf, 8)) +\
			pow(t, 7) * (TRAJ_7 * p0 / pow(tf, 7) - TRAJ_7 * pf / pow(tf, 7)) +\
			pow(t, 6) * (TRAJ_6 * p0 / pow(tf, 6) - TRAJ_6 * pf / pow(tf, 6)) +\
			pow(t, 5) * (TRAJ_5 * p0 / pow(tf, 5) - TRAJ_5 * pf / pow(tf, 5));

		return p;
}

//------------------------------------------------------------------------------
//	Function trajectory_velocity
// 	returns the velocity v of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_velocity(float t, float tf, float p0, float pf)
{
float	v;
		v = 9 * pow(t, 8) * (TRAJ_9 * p0 / pow(tf, 9) - TRAJ_9 * pf / pow(tf, 9)) +\
			8 * pow(t, 7) * (TRAJ_8 * p0 / pow(tf, 8) - TRAJ_8 * pf / pow(tf, 8)) +\
			7 * pow(t, 6) * (TRAJ_7 * p0 / pow(tf, 7) - TRAJ_7 * pf / pow(tf, 7)) +\
			6 * pow(t, 5) * (TRAJ_6 * p0 / pow(tf, 6) - TRAJ_6 * pf / pow(tf, 6)) +\
			5 * pow(t, 4) * (TRAJ_5 * p0 / pow(tf, 5) - TRAJ_5 * pf / pow(tf, 5));

		return v;
}

//------------------------------------------------------------------------------
//	Function trajectory_acceleration
// 	returns the acceleration a of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_acceleration(float t, float tf, float p0, float pf)
{
float	a;
		a = 9 * 8 * pow(t, 7) * (TRAJ_9 * p0 / pow(tf, 9) - TRAJ_9 * pf / pow(tf, 9)) +\
			8 * 7 * pow(t, 6) * (TRAJ_8 * p0 / pow(tf, 8) - TRAJ_8 * pf / pow(tf, 8)) +\
			7 * 6 * pow(t, 5) * (TRAJ_7 * p0 / pow(tf, 7) - TRAJ_7 * pf / pow(tf, 7)) +\
			6 * 5 * pow(t, 4) * (TRAJ_6 * p0 / pow(tf, 6) - TRAJ_6 * pf / pow(tf, 6)) +\
			5 * 4 * pow(t, 3) * (TRAJ_5 * p0 / pow(tf, 5) - TRAJ_5 * pf / pow(tf, 5));

		return a;
}

//------------------------------------------------------------------------------
//	Function trajectory_jerk
// 	returns the jerk a of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_jerk(float t, float tf, float p0, float pf)
{
float	j;
		j = 9 * 8 * 7 * pow(t, 6) * (TRAJ_9 * p0 / pow(tf, 9) - TRAJ_9 * pf / pow(tf, 9)) +\
			8 * 7 * 6 * pow(t, 5) * (TRAJ_8 * p0 / pow(tf, 8) - TRAJ_8 * pf / pow(tf, 8)) +\
			7 * 6 * 5 * pow(t, 4) * (TRAJ_7 * p0 / pow(tf, 7) - TRAJ_7 * pf / pow(tf, 7)) +\
			6 * 5 * 4 * pow(t, 3) * (TRAJ_6 * p0 / pow(tf, 6) - TRAJ_6 * pf / pow(tf, 6)) +\
			5 * 4 * 3 * pow(t, 2) * (TRAJ_5 * p0 / pow(tf, 5) - TRAJ_5 * pf / pow(tf, 5));

		return j;
}

//------------------------------------------------------------------------------
//	Function trajectory_jerk_derivative
// 	returns the jerk derivative a of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_jerk_derivative(float t, float tf, float p0, float pf)
{
float	dj;
		dj = 9 * 8 * 7 * 6 * pow(t, 5) * (TRAJ_9 * p0 / pow(tf, 9) - TRAJ_9 * pf / pow(tf, 9)) +\
			8 * 7 * 6 * 5 * pow(t, 4) * (TRAJ_8 * p0 / pow(tf, 8) - TRAJ_8 * pf / pow(tf, 8)) +\
			7 * 6 * 5 * 4 * pow(t, 3) * (TRAJ_7 * p0 / pow(tf, 7) - TRAJ_7 * pf / pow(tf, 7)) +\
			6 * 5 * 4 * 3 * pow(t, 2) * (TRAJ_6 * p0 / pow(tf, 6) - TRAJ_6 * pf / pow(tf, 6)) +\
			5 * 4 * 3 * 2 * t * (TRAJ_5 * p0 / pow(tf, 5) - TRAJ_5 * pf / pow(tf, 5));

		return dj;
}

//------------------------------------------------------------------------------
//	Function pitch
// 	returns the pitch of the 2D rigid body along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	pitch(float t, float tf, float x0, float y0, float xf, float yf)
{
float	th;
		th = atan2(-trajectory_acceleration(t, tf, x0, xf),\
			trajectory_acceleration(t, tf, y0, yf) + G);

		return th;
}

//------------------------------------------------------------------------------
//	Function pitch rate
// 	returns the pitch rate of the 2D rigid body along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	pitch_rate(float t, float tf, float x0, float y0, float xf, float yf)
{
float	dth;
		dth = (-(trajectory_acceleration(t, tf, y0, yf) + G) *\
			trajectory_jerk(t, tf, x0, xf) +\
			trajectory_acceleration(t, tf, x0, xf) *\
			trajectory_jerk(t, tf, y0, yf)) /\
			(pow(trajectory_acceleration(t, tf, x0, xf), 2) +\
			pow(trajectory_acceleration(t, tf, y0, yf) + G, 2));

		return dth;
}

//------------------------------------------------------------------------------
//	Function pitch acceleration
// 	returns the pitch acceleration of the 2D rigid body along the desired
//	trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	pitch_acceleration(float t, float tf, float x0, float y0, float xf, float yf)
{
float	ddth;
		ddth = 1 / pow(pow(trajectory_acceleration(t, tf, x0, xf), 2) +\
			pow(trajectory_acceleration(t, tf, y0, yf) + G, 2), 2) *\
			(- pow(G + trajectory_acceleration(t, tf, y0, yf), 2) *\
			(- 2 * trajectory_jerk(t, tf, x0, xf) * trajectory_jerk(t, tf, y0, yf) +\
			(trajectory_acceleration(t, tf, y0, yf) + G) *\
			trajectory_jerk_derivative(t, tf, x0, xf)) -\
			pow(trajectory_acceleration(t, tf, x0, xf), 2) *\
			(2 * trajectory_jerk(t, tf, x0, xf) * trajectory_jerk(t, tf, y0, yf) +\
			(trajectory_acceleration(t, tf, y0, yf) + G) *\
			trajectory_jerk_derivative(t, tf, x0, xf)) +\
			pow(trajectory_acceleration(t, tf, x0, xf), 3) *\
			trajectory_jerk_derivative(t, tf, y0, yf) +\
			trajectory_acceleration(t, tf, x0, xf) *\
			(trajectory_acceleration(t, tf, y0, yf) + G) *\
			(2 * pow(trajectory_jerk(t, tf, x0, xf), 2) -\
			2 * pow(trajectory_jerk(t, tf, y0, yf), 2) +\
			(trajectory_acceleration(t, tf, y0, yf) + G) *\
			trajectory_jerk_derivative(t, tf, y0, yf)));

		return ddth;
}

//------------------------------------------------------------------------------
//	Function left_force
// 	returns the force generated by the left propeller of the 2D quadcopter
//	along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	left_force(float t, float tf, float x0, float y0, float xf, float yf)
{
float	fl;
		fl = - IZ / (2 * L) * pitch_acceleration(t, tf, x0, y0, xf, yf) -\
			M / 2 * (trajectory_acceleration(t, tf, x0, xf) *\
			sin(pitch(t, tf, x0, y0, xf, yf)) -\
			trajectory_acceleration(t, tf, y0, yf) *\
			cos(pitch(t, tf, x0, y0, xf, yf)) -\
			G *  cos(pitch(t, tf, x0, y0, xf, yf)));

		return fl;
}


//------------------------------------------------------------------------------
//	Function right_force
// 	returns the force generated by the right propeller of the 2D quadcopter
//	along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	right_force(float t, float tf, float x0, float y0, float xf, float yf)
{
float	fr;
		fr = IZ / (2 * L) * pitch_acceleration(t, tf, x0, y0, xf, yf) -\
			M / 2 * (trajectory_acceleration(t, tf, x0, xf) *\
			sin(pitch(t, tf, x0, y0, xf, yf)) -\
			trajectory_acceleration(t, tf, y0, yf) *\
			cos(pitch(t, tf, x0, y0, xf, yf)) -\
			G *  cos(pitch(t, tf, x0, y0, xf, yf)));

		return fr;
}

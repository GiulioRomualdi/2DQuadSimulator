//------------------------------------------------------------------------------
//	SIMULATOR.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <math.h>

//------------------------------------------------------------------------------
//	TRAJECTORY GENERATION CONSTANTS
//------------------------------------------------------------------------------
#define	TRAJ_9	-70		// 9-th order coefficient of trajectory polynomial
#define TRAJ_8	315		// 8-th order coefficient of trajectory polynomial
#define TRAJ_7	-540	// 7-th order coefficient of trajectory polynomial
#define TRAJ_6	420		// 6-th order coefficient of trajectory polynomial
#define TRAJ_5	-126	// 5-th order coefficient of trajectory polynomial

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

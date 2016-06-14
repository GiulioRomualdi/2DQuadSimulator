//------------------------------------------------------------------------------
//	SIMULATOR.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <math.h>

//------------------------------------------------------------------------------
//	2D QUADCOPTER CONSTANTS
//------------------------------------------------------------------------------
#define M		0.3					// Quadcopter mass in Kg
#define L		0.15				// Quadcopter length in m
#define IZ		(M * L * L / 12)	// Moment of inertia around axis x
#define G		9.81				// Gravitational acceleration in m/s^2

//------------------------------------------------------------------------------
//	TRAJECTORY GENERATION CONSTANTS
//------------------------------------------------------------------------------
#define	TRAJ_9	-70					// 9-th coefficient of trajectory polynomial
#define TRAJ_8	315					// 8-th coefficient of trajectory polynomial
#define TRAJ_7	-540				// 7-th coefficient of trajectory polynomial
#define TRAJ_6	420					// 6-th coefficient of trajectory polynomial
#define TRAJ_5	-126				// 5-th coefficient of trajectory polynomial

//------------------------------------------------------------------------------
//	Function trajectory
// 	returns the point along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory(float t, float tf, float p0, float pf)
{
float	pow_tf_5, pow_tf_6, pow_tf_7, pow_tf_8, pow_tf_9;
float 	pow_t_5, pow_t_6, pow_t_7, pow_t_8, pow_t_9;
float 	p;

		pow_tf_5 = pow(tf, 5);
		pow_tf_6 = pow_tf_5 * tf;
		pow_tf_7 = pow_tf_6 * tf;
		pow_tf_8 = pow_tf_7 * tf;
		pow_tf_9 = pow_tf_8 * tf;

		pow_t_5 = pow(t, 5);
		pow_t_6 = pow_t_5 * t;
		pow_t_7 = pow_t_6 * t;
		pow_t_8 = pow_t_7 * t;
		pow_t_9 = pow_t_8 * t; 

		p = p0 +\
			pow_t_9 * (TRAJ_9 * p0 / pow_tf_9 - TRAJ_9 * pf / pow_tf_9) +\
			pow_t_8 * (TRAJ_8 * p0 / pow_tf_8 - TRAJ_8 * pf / pow_tf_8) +\
			pow_t_7 * (TRAJ_7 * p0 / pow_tf_7 - TRAJ_7 * pf / pow_tf_7) +\
			pow_t_6 * (TRAJ_6 * p0 / pow_tf_6 - TRAJ_6 * pf / pow_tf_6) +\
			pow_t_5 * (TRAJ_5 * p0 / pow_tf_5 - TRAJ_5 * pf / pow_tf_5);

		return p;
}

//------------------------------------------------------------------------------
//	Function trajectory_velocity
// 	returns the velocity of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_velocity(float t, float tf, float p0, float pf)
{
float	pow_tf_5, pow_tf_6, pow_tf_7, pow_tf_8, pow_tf_9;
float 	pow_t_4, pow_t_5, pow_t_6, pow_t_7, pow_t_8;
float 	v;

		pow_tf_5 = pow(tf, 5);
		pow_tf_6 = pow_tf_5 * tf;
		pow_tf_7 = pow_tf_6 * tf;
		pow_tf_8 = pow_tf_7 * tf;
		pow_tf_9 = pow_tf_8 * tf;

		pow_t_4 = pow(t, 4);
		pow_t_5 = pow_t_4 * t;
		pow_t_6 = pow_t_5 * t;
		pow_t_7 = pow_t_6 * t;
		pow_t_8 = pow_t_7 * t;

		v = 9 * pow_t_8 * (TRAJ_9 * p0 / pow_tf_9 - TRAJ_9 * pf / pow_tf_9) +\
			8 * pow_t_7 * (TRAJ_8 * p0 / pow_tf_8 - TRAJ_8 * pf / pow_tf_8) +\
			7 * pow_t_6 * (TRAJ_7 * p0 / pow_tf_7 - TRAJ_7 * pf / pow_tf_7) +\
			6 * pow_t_5 * (TRAJ_6 * p0 / pow_tf_6 - TRAJ_6 * pf / pow_tf_6) +\
			5 * pow_t_4 * (TRAJ_5 * p0 / pow_tf_5 - TRAJ_5 * pf / pow_tf_5);

		return v;
}

//------------------------------------------------------------------------------
//	Function trajectory_acceleration
// 	returns the acceleration of a point along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_acceleration(float t, float tf, float p0, float pf)
{
float	pow_tf_5, pow_tf_6, pow_tf_7, pow_tf_8, pow_tf_9;
float 	pow_t_3, pow_t_4, pow_t_5, pow_t_6, pow_t_7;
float 	a;

		pow_tf_5 = pow(tf, 5);
		pow_tf_6 = pow_tf_5 * tf;
		pow_tf_7 = pow_tf_6 * tf;
		pow_tf_8 = pow_tf_7 * tf;
		pow_tf_9 = pow_tf_8 * tf;

		pow_t_3 = pow(t, 3);
		pow_t_4 = pow_t_3 * t;
		pow_t_5 = pow_t_4 * t;
		pow_t_6 = pow_t_5 * t;
		pow_t_7 = pow_t_6 * t;

		a = 9 * 8 * pow_t_7 * (TRAJ_9 * p0 / pow_tf_9 - TRAJ_9 * pf / pow_tf_9) +\
			8 * 7 * pow_t_6 * (TRAJ_8 * p0 / pow_tf_8 - TRAJ_8 * pf / pow_tf_8) +\
			7 * 6 * pow_t_5 * (TRAJ_7 * p0 / pow_tf_7 - TRAJ_7 * pf / pow_tf_7) +\
			6 * 5 * pow_t_4 * (TRAJ_6 * p0 / pow_tf_6 - TRAJ_6 * pf / pow_tf_6) +\
			5 * 4 * pow_t_3 * (TRAJ_5 * p0 / pow_tf_5 - TRAJ_5 * pf / pow_tf_5);

		return a;
}

//------------------------------------------------------------------------------
//	Function trajectory_jerk
// 	returns the jerk (third derivative of position) of a point 
//	along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_jerk(float t, float tf, float p0, float pf)
{
float	pow_tf_5, pow_tf_6, pow_tf_7, pow_tf_8, pow_tf_9;
float 	pow_t_2, pow_t_3, pow_t_4, pow_t_5, pow_t_6;
float 	j;

		pow_tf_5 = pow(tf, 5);
		pow_tf_6 = pow_tf_5 * tf;
		pow_tf_7 = pow_tf_6 * tf;
		pow_tf_8 = pow_tf_7 * tf;
		pow_tf_9 = pow_tf_8 * tf;

		pow_t_2 = pow(t, 2);
		pow_t_3 = pow_t_2 * t;
		pow_t_4 = pow_t_3 * t;
		pow_t_5 = pow_t_4 * t;
		pow_t_6 = pow_t_5 * t;

		j = 9 * 8 * 7 * pow_t_6 * (TRAJ_9 * p0 / pow_tf_9 - TRAJ_9 * pf / pow_tf_9) +\
			8 * 7 * 6 * pow_t_5 * (TRAJ_8 * p0 / pow_tf_8 - TRAJ_8 * pf / pow_tf_8) +\
			7 * 6 * 5 * pow_t_4 * (TRAJ_7 * p0 / pow_tf_7 - TRAJ_7 * pf / pow_tf_7) +\
			6 * 5 * 4 * pow_t_3 * (TRAJ_6 * p0 / pow_tf_6 - TRAJ_6 * pf / pow_tf_6) +\
			5 * 4 * 3 * pow_t_2 * (TRAJ_5 * p0 / pow_tf_5 - TRAJ_5 * pf / pow_tf_5);

		return j;
}

//------------------------------------------------------------------------------
//	Function trajectory_jounce
// 	returns the jounce (fourth derivative of position) of a point 
//	along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
float	trajectory_jounce(float t, float tf, float p0, float pf)
{
float	pow_tf_5, pow_tf_6, pow_tf_7, pow_tf_8, pow_tf_9;
float 	pow_t_2, pow_t_3, pow_t_4, pow_t_5;
float 	j;

		pow_tf_5 = pow(tf, 5);
		pow_tf_6 = pow_tf_5 * tf;
		pow_tf_7 = pow_tf_6 * tf;
		pow_tf_8 = pow_tf_7 * tf;
		pow_tf_9 = pow_tf_8 * tf;

		pow_t_2 = pow(t, 2);
		pow_t_3 = pow_t_2 * t;
		pow_t_4 = pow_t_3 * t;
		pow_t_5 = pow_t_4 * t;

		j = 9 * 8 * 7 * 6 * pow_t_5 * (TRAJ_9 * p0 / pow_tf_9 - TRAJ_9 * pf / pow_tf_9) +\
			8 * 7 * 6 * 5 * pow_t_4 * (TRAJ_8 * p0 / pow_tf_8 - TRAJ_8 * pf / pow_tf_8) +\
			7 * 6 * 5 * 4 * pow_t_3 * (TRAJ_7 * p0 / pow_tf_7 - TRAJ_7 * pf / pow_tf_7) +\
			6 * 5 * 4 * 3 * pow_t_2 * (TRAJ_6 * p0 / pow_tf_6 - TRAJ_6 * pf / pow_tf_6) +\
			5 * 4 * 3 * 2 * t * (TRAJ_5 * p0 / pow_tf_5 - TRAJ_5 * pf / pow_tf_5);

		return j;
}

//------------------------------------------------------------------------------
//	Function pitch
// 	returns the pitch angle of the 2D rigid body along the desired trajectory
//	given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	pitch(float t, float tf, float x0, float y0, float xf, float yf)
{
float	x_acc, y_acc;
float	p;

		x_acc = trajectory_acceleration(t, tf, x0, xf);
		y_acc = trajectory_acceleration(t, tf, y0, yf);

		p = atan2(-x_acc, y_acc + G);

		return p;
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
float	x_acc, y_acc;
float 	x_jerk, y_jerk;
float	r;

		x_acc = trajectory_acceleration(t, tf, x0, xf);
		y_acc = trajectory_acceleration(t, tf, y0, yf);
		x_jerk = trajectory_jerk(t, tf, x0, xf);
		y_jerk = trajectory_jerk(t, tf, y0, yf);

		r = (-(y_acc + G) *	x_jerk + x_acc * y_jerk) /\
			(pow(x_acc, 2) + pow(y_acc + G, 2));

		return r;
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
float	x_acc, y_acc;
float 	x_jerk, y_jerk;
float 	x_jounce, y_jounce;
float	a;

		x_acc = trajectory_acceleration(t, tf, x0, xf);
		y_acc = trajectory_acceleration(t, tf, y0, yf);
		x_jerk = trajectory_jerk(t, tf, x0, xf);
		y_jerk = trajectory_jerk(t, tf, y0, yf);
		x_jounce = trajectory_jounce(t, tf, x0, xf);
		y_jounce = trajectory_jounce(t, tf, y0, yf);

		a = 1 / pow(pow(x_acc, 2) + pow(y_acc + G, 2), 2) * (- pow(G + y_acc, 2) *\
			(- 2 * x_jerk * y_jerk + (y_acc + G) * x_jounce) - pow(x_acc, 2) *\
			(2 * x_jerk * y_jerk + (y_acc + G) * x_jounce) + pow(x_acc, 3) *\
			y_jounce + x_acc * (y_acc + G) * (2 * pow(x_jerk, 2) -\
			2 * pow(y_jerk, 2) + (y_acc + G) * y_jounce));

		return a;
}

//------------------------------------------------------------------------------
//	Function force_left
// 	returns the force generated by the left propeller of the 2D quadcopter
//	along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	force_left(float t, float tf, float x0, float y0, float xf, float yf)
{
float 	x_acc, y_acc;
float 	pitch_, pitch_acc;
float	f;

		x_acc = trajectory_acceleration(t, tf, x0, xf);
		y_acc = trajectory_acceleration(t, tf, y0, yf);

		pitch_ = pitch(t, tf, x0, xf, y0, yf);
		pitch_acc = pitch_acceleration(t, tf, x0, y0, xf, yf);

		f = - IZ / (2 * L) * pitch_acc -\
			M / 2 * (x_acc * sin(pitch_) - y_acc * cos(pitch_) -\
			G * cos(pitch_));

		return f;
}


//------------------------------------------------------------------------------
//	Function force_right
// 	returns the force generated by the right propeller of the 2D quadcopter
//	along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final coordinates (x0, y0, xf, yf)
//------------------------------------------------------------------------------
float	force_right(float t, float tf, float x0, float y0, float xf, float yf)
{
float 	x_acc, y_acc;
float 	pitch_, pitch_acc;
float	f;

		x_acc = trajectory_acceleration(t, tf, x0, xf);
		y_acc = trajectory_acceleration(t, tf, y0, yf);

		pitch_ = pitch(t, tf, x0, xf, y0, yf);
		pitch_acc = pitch_acceleration(t, tf, x0, y0, xf, yf);

		f = IZ / (2 * L) * pitch_acc -\
		 	M / 2 * (x_acc * sin(pitch_) - y_acc * cos(pitch_) -\
			G *  cos(pitch_));

		return f;
}

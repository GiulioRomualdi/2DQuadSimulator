//------------------------------------------------------------------------------
//	SIMULATOR.C
//------------------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include "simulator.h"
#include "utils.h"

//------------------------------------------------------------------------------
//	TRAJECTORY GENERATION CONSTANTS
//------------------------------------------------------------------------------
#define	TRAJ_9			-70.0				// 9-th coeff. of trajectory polynomial
#define TRAJ_8			315.0				// 8-th coeff. of trajectory polynomial
#define TRAJ_7			-540.0				// 7-th coeff. of trajectory polynomial
#define TRAJ_6			420.0				// 6-th coeff. of trajectory polynomial
#define TRAJ_5			-126.0				// 5-th coeff. of trajectory polynomial
//------------------------------------------------------------------------------
//	STANDARD DEVIATION CONSTANTS
//------------------------------------------------------------------------------
#define	X0_STD			1.0 / 3.0			// std of x0
#define VX0_STD			1.0 / 3.0			// std of vx0
#define Y0_STD			1.0 / 3.0			// std of y0
#define VY0_STD			1.0 / 3.0			// std of vy0
#define THETA0_STD		0.6 / 3.0			// std of theta0
#define VTHETA0_STD		0.0					// std of vtheta0
#define	WIND_X_STD 		1.0 / 3.0			// std of the force due to wind along x
#define WIND_Y_STD		1.0 / 3.0			// std of the force due to wind along y
#define NOISE_X_STD		1.0 / 3.0			// std of x measurament noise
#define NOISE_Y_STD		1.0 / 3.0			// std of y measurament noise
#define NOISE_THETA_STD	0.6 / 3.0			// std of theta measurament noise
//------------------------------------------------------------------------------
//	GUIDANCE SYSTEMS CONSTANTS
//------------------------------------------------------------------------------
#define	GUID_WAIT_TIME	2.0					// wait time for new target election

//------------------------------------------------------------------------------
//	GLOABAL VARIABLE DEFINITIONS
//------------------------------------------------------------------------------

// Discrete Linear Quadratic (LQ) gain matrix
float	K_LQ[2][6] = {{-1.5295, -2.2586, -15.1509, -15.7189, 8.7459, 1.6575},
					  {1.5295, 2.2586, -15.1509, -15.7189, -8.7459, -1.6575}};

int		guid_switches[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	GLOABAL DATA STRUCTURES DEFINITIONS
//------------------------------------------------------------------------------
struct state states[MAX_QUADROTORS];
struct state desired_trajectories[MAX_QUADROTORS];
struct kalman_state kalman_states[MAX_QUADROTORS];
struct trajectory_state traj_states[MAX_QUADROTORS];
struct force forces[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable 'guid_switch'
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function init_selected_quad
//	initializes the variable guid_switches
//------------------------------------------------------------------------------
void	init_guidance_switches()
{
int		i;

		for (i = 0; i < MAX_QUADROTORS; i++)
			guid_switches[i] = 0;
}

//------------------------------------------------------------------------------
//	Function get_guidance_state
//	returns the state of the guidance system of the i-th quadrotor
//------------------------------------------------------------------------------
int		get_guidance_state(int index)
{
int		value;

		pthread_mutex_lock(&guid_switches_mutex[index]);
		value = guid_switches[index];
		pthread_mutex_unlock(&guid_switches_mutex[index]);

		return value;
}

//------------------------------------------------------------------------------
//	Function switch_guidance
//	turns on/off the guidance system of the i-th quadrotor
//------------------------------------------------------------------------------
void	switch_guidance(int index, int active)
{
		pthread_mutex_lock(&guid_switches_mutex[index]);
		guid_switches[index] = active;
		pthread_mutex_unlock(&guid_switches_mutex[index]);
}

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable with type 'trajectory_state'
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function copy_traj_param
//------------------------------------------------------------------------------
void	copy_traj_param(struct trajectory_state *src,\
						struct trajectory_state *dst,\
						pthread_mutex_t *mutex)
{
		pthread_mutex_lock(mutex);
		*dst = *src;
		pthread_mutex_unlock(mutex);
}

//------------------------------------------------------------------------------
//	Function update_trajectory_time
//	update the current time of the trajectory
//------------------------------------------------------------------------------
static
void	update_trajectory_time(int i, float time_step)
{
		pthread_mutex_lock(&guidance_mutex[i]);
		traj_states[i].current_time += time_step;
		pthread_mutex_unlock(&guidance_mutex[i]);
}

//------------------------------------------------------------------------------
//	Function set_traj_params
//	set the trajectory parameters for the i-th quadrotor
//	given the coordinates of the new target
//------------------------------------------------------------------------------
void	set_traj_params(int i, float x, float y)
{
struct trajectory_state traj_new_p;
state	current_state;
float 	tf;

		// Get the current state of the quadrotor
		copy_state(&(kalman_states[i].estimate), &current_state, &kalman_mutex[i]);

		// Set new trajectory parameters
		traj_new_p.x0 = current_state.x;
		traj_new_p.y0 = current_state.y;
		traj_new_p.xf = x;
		traj_new_p.yf = y;

		tf = sqrt(pow(traj_new_p.xf - traj_new_p.x0, 2) + pow(traj_new_p.yf - traj_new_p.y0, 2));
		// Not too short
		if (tf < 2)
			tf = 2;
		// Not too long
		if (tf > 5)
			tf = 5;

		traj_new_p.current_time = 0;
		traj_new_p.final_time = tf;

		// Update guidance parameters
		copy_traj_param(&traj_new_p, &traj_states[i], &guidance_mutex[i]);
}

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable with type 'state'
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function copy_state
//------------------------------------------------------------------------------
void 	copy_state(state *src, state *dst, pthread_mutex_t *mutex)
{
		pthread_mutex_lock(mutex);
		*dst = *src;
		pthread_mutex_unlock(mutex);
}

//------------------------------------------------------------------------------
//	Function copy_forces
//------------------------------------------------------------------------------
static
void 	copy_forces(float src[2], struct force *dst,  pthread_mutex_t *mutex)
{
		pthread_mutex_lock(mutex);
		dst->force_left = src[0];
		dst->force_right = src[1];
		pthread_mutex_unlock(mutex);
}

//------------------------------------------------------------------------------
//	TRAJECTORY GENERATION FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function trajectory
// 	returns the point along the desired trajectory given the current time t,
// 	the time of flight tf and
//	the initial and final position (p0, pf)
//------------------------------------------------------------------------------
static
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
static
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
static
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
static
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
static
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
static
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
static
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
static
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
static
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
static
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

//------------------------------------------------------------------------------
//	Function get_reference_state
//	evaluate the current reference state for the i-th quadrotor
//	store the result in the 'state' and 'input'
//------------------------------------------------------------------------------
static
void	get_reference_state(int i, float time_step, state* state_, float input[2])
{
struct trajectory_state traj_p;

		copy_traj_param(&traj_states[i], &traj_p, &guidance_mutex[i]);

		if (traj_p.current_time >= traj_p.final_time)
			traj_p.current_time = traj_p.final_time;

		// Store the state
		state_->x = trajectory(traj_p.current_time, traj_p.final_time,\
							   traj_p.x0, traj_p.xf);
		state_->vx = trajectory_velocity(traj_p.current_time, traj_p.final_time,\
										 traj_p.x0, traj_p.xf);
		state_->y = trajectory(traj_p.current_time, traj_p.final_time,\
							   traj_p.y0, traj_p.yf);
		state_->vy = trajectory_velocity(traj_p.current_time, traj_p.final_time,\
										 traj_p.y0, traj_p.yf);
		state_->theta = pitch(traj_p.current_time, traj_p.final_time,\
							  traj_p.x0, traj_p.y0, traj_p.xf, traj_p.yf);
		state_->vtheta = pitch_rate(traj_p.current_time, traj_p.final_time,\
									traj_p.x0, traj_p.y0, traj_p.xf, traj_p.yf);

		// Store the input
		input[0] = force_left(traj_p.current_time, traj_p.final_time,\
							  traj_p.x0, traj_p.y0, traj_p.xf, traj_p.yf);
		input[1] = force_right(traj_p.current_time, traj_p.final_time,\
							   traj_p.x0, traj_p.y0, traj_p.xf, traj_p.yf);
}

//------------------------------------------------------------------------------
// 	SYSTEM DYNAMICS, SYSTEM IO, SYSTEM FEEDBACK FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function init_state
//------------------------------------------------------------------------------
void 	init_state(int i, float x, float vx, float y,\
				   float vy, float theta, float vtheta)
{
		states[i].x = x;
		states[i].vx = vx;
		states[i].y = y;
		states[i].vy = vy;
		states[i].theta = theta;
		states[i].vtheta = vtheta;
}

//------------------------------------------------------------------------------
//	Function dynamics
//	evaluates the discrete time dynamics of the 2D quadrotor
//	given the inputs 'fl' and 'fr', the disturbaces due to wind,
//	the sampling time 'T' and the previous state 'src'
//	puts the result in 'dest'
//------------------------------------------------------------------------------
static
void	dynamics(float T, float fl, float fr, float wind_x, float wind_y,\
				 state src, state* dest)
{
 		dest->x			= src.x + src.vx * T;
		dest->vx 		= src.vx + (- sin(src.theta) * (fl + fr) + wind_x) * T / M;
		dest->y			= src.y + src.vy * T;
		dest->vy		= src.vy + (- M * G +  cos(src.theta) * (fl + fr) + wind_y) * T / M;
		dest->theta		= src.theta + src.vtheta * T;
		dest->vtheta	= src.vtheta + L / IZ * (fr - fl) * T;
}

//------------------------------------------------------------------------------
//	Function system_io
//	evaluates the input/output behavior of the system taking into account
//	noisy disturbances and mesasure
//	puts the evaluated state in 'state' and the evaluated measures in 'mesasure'
//------------------------------------------------------------------------------
static
void	system_io(int i, float T, float fl, float fr, state* state, float measure[3])
{
float	wind_x, wind_y, sigma_wind_x, sigma_wind_y;
float	x_m, y_m, theta_m, sigma_x_m, sigma_y_m, sigma_theta_m;

		// Generate wind forces according to the process noise covariance matrix Q
		sigma_wind_x = kalman_states[i].Q[0][0];
		sigma_wind_y = kalman_states[i].Q[1][1];
		wind_x = get_gaussian(sigma_wind_x);
		wind_y = get_gaussian(sigma_wind_y);

		// Evaluate the dynamics
		dynamics(T, fl, fr, wind_x, wind_y, *state, state);

		// Add noise to measure according to measure noise covariance matrix R
		sigma_x_m = kalman_states[i].R[0][0];
		sigma_y_m = kalman_states[i].R[1][1];
		sigma_theta_m = kalman_states[i].R[2][2];
		x_m = state->x + get_gaussian(sigma_x_m);
		y_m = state->y + get_gaussian(sigma_y_m);
		theta_m = state->theta + get_gaussian(sigma_theta_m);
		measure[0] = x_m;
		measure[1] = y_m;
		measure[2] = theta_m;
}

//------------------------------------------------------------------------------
//	Function feedback_control
//	evaluates the output of the feedback controller
//------------------------------------------------------------------------------
static
void	feedback_control(state current_state, state desired_trajectory,\
						 float nominal_fl, float nominal_fr, float output[2])
{
float	error[6][1];
float	du[2][1];

		// Evaluate the trajectory error
		error[0][0] = current_state.x - desired_trajectory.x;
		error[1][0] = current_state.vx - desired_trajectory.vx;
		error[2][0] = current_state.y - desired_trajectory.y;
		error[3][0] = current_state.vy - desired_trajectory.vy;
		error[4][0] = current_state.theta - desired_trajectory.theta;
		error[5][0] = current_state.vtheta - desired_trajectory.vtheta;

		// Evaluate the the control output wrt the nominal control
		// du = K_LQ * error
		mat_mul(2, 6, K_LQ, 6, 1, error, du);
		// Add the nominal control
		output[0] = du[0][0] + nominal_fl;
		output[1] = du[1][0] + nominal_fr;
}

//------------------------------------------------------------------------------
//	EXTENDED KALMAN FILTER FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function init_A
//	initializes the i-th state Jacobian matrix given the sampling time T
//------------------------------------------------------------------------------
static
void 	init_A(int i, float T)
{
int		j;

		mat_zero(6, 6, kalman_states[i].A);

		for (j = 0; j < 6; j++)
			kalman_states[i].A[j][j] = 1;

		kalman_states[i].A[0][1] = T;
		kalman_states[i].A[2][3] = T;
		kalman_states[i].A[4][5] = T;
}

//------------------------------------------------------------------------------
//	Function update_A
//	update the i-th state Jacobian matrix given the sampling time T,
//	the a priori estimate of the pitch angle a_priori_theta
//	and the right and left forces
//------------------------------------------------------------------------------
static
void 	update_A(int i, float T, float a_priori_theta, float fl, float fr)
{
		kalman_states[i].A[1][4] = - (fl + fr) * T / M * cos(a_priori_theta);
		kalman_states[i].A[3][4] = - (fl + fr) * T / M * sin(a_priori_theta);
}

//------------------------------------------------------------------------------
//	Function init_W
//	initializes the i-th noise input Jacobian matrix
//------------------------------------------------------------------------------
static
void 	init_W(int i, float T)
{
		mat_zero(6, 3, kalman_states[i].W);

		kalman_states[i].W[1][0] = T / M;
		kalman_states[i].W[3][1] = T / M;
		kalman_states[i].W[5][2] = T / IZ;
}

//------------------------------------------------------------------------------
//	Function init_C
//	initializes the i-th output Jacobian matrix
//------------------------------------------------------------------------------
static
void 	init_C(int i)
{
int 	j;
		mat_zero(3, 6, kalman_states[i].C);

		for (j = 0; j < 3; j++)
			kalman_states[i].C[j][2*j] = 1;
}

//------------------------------------------------------------------------------
//	Function init_P
//	initializes the i-th error state covariance matrix
//	given the variances of the state
//------------------------------------------------------------------------------
static
void 	init_P(int i, float sigma_x, float sigma_vx,\
				float sigma_y, float sigma_vy,\
				float sigma_theta, float sigma_vtheta)
{
		mat_zero(6, 6, kalman_states[i].P);

		kalman_states[i].P[0][0] = sigma_x;
		kalman_states[i].P[1][1] = sigma_vx;
		kalman_states[i].P[2][2] = sigma_y;
		kalman_states[i].P[3][3] = sigma_vy;
		kalman_states[i].P[4][4] = sigma_theta;
		kalman_states[i].P[5][5] = sigma_vtheta;
}

//------------------------------------------------------------------------------
//	Function init_Q
//	initializes the i-th process noise covariance matrix
//------------------------------------------------------------------------------
static
void 	init_Q(int i, float sigma_wind_x, float sigma_wind_y)
{
		mat_zero(2, 2, kalman_states[i].Q);

		kalman_states[i].Q[0][0] = sigma_wind_x;
		kalman_states[i].Q[1][1] = sigma_wind_y;
}

//------------------------------------------------------------------------------
//	Function init_R
//	initializes the i-th measurement noise covariance matrix
//------------------------------------------------------------------------------
static
void 	init_R(int i, float sigma_x, float sigma_y, float sigma_theta)
{
		mat_zero(3, 3, kalman_states[i].R);

		kalman_states[i].R[0][0] = sigma_x;
		kalman_states[i].R[1][1] = sigma_y;
		kalman_states[i].R[2][2] = sigma_theta;
}

//------------------------------------------------------------------------------
//	Function init_state_estimate
//	initializes the i-th state estimate
//------------------------------------------------------------------------------
static
void 	init_state_estimate(int i, float x, float vx, float y,\
							float vy, float theta, float vtheta)
{
		kalman_states[i].estimate.x = x + get_gaussian(X0_STD);
		kalman_states[i].estimate.vx = vx + get_gaussian(VX0_STD);
		kalman_states[i].estimate.y = y + get_gaussian(Y0_STD);
		kalman_states[i].estimate.vy = vy + get_gaussian(VY0_STD);
		kalman_states[i].estimate.theta = theta + get_gaussian(THETA0_STD);
		kalman_states[i].estimate.vtheta = vtheta + get_gaussian(VTHETA0_STD);
}

//------------------------------------------------------------------------------
//	Function init_ekf_matrix
//	initializes the matrices of the i-th kalman filter
//------------------------------------------------------------------------------
static
void	init_ekf_matrix(int i, float T)
{
		init_A(i, T);
		init_W(i, T);
		init_C(i);
		init_P(i, X0_STD, VX0_STD, Y0_STD, VY0_STD, THETA0_STD, VTHETA0_STD);
		init_Q(i, WIND_X_STD, WIND_Y_STD);
		init_R(i, NOISE_X_STD, NOISE_Y_STD, NOISE_THETA_STD);
}

//------------------------------------------------------------------------------
//	Function ekf
//	Extended Kalman Filter
//------------------------------------------------------------------------------
static
void	ekf(int i, float T, float fl, float fr, float x_m, float y_m, float theta_m,
			state* estimate)
{
state	state_priori;
float 	P1[6][6], P2[6][6], P2_part[6][2], A_t[6][6], W_t[2][6];
float	K1[6][3], K2[3][3], K2_part[3][6], C_t[6][3];
float	innovation[3][1], weigthed_innovation[6][1];
float	wind_x, wind_y;

	//---------------------------------------------------------------------
	// A PRIORI STAGE
	//---------------------------------------------------------------------
	//
		// propagate dynamics using mean input disturbances
		wind_x = 0;
 		wind_y = 0;
		dynamics(T, fl, fr, wind_x, wind_y, kalman_states[i].estimate, &state_priori);

		// update state Jacobian A
		update_A(i, T, state_priori.theta, fl, fr);

		// prediction of error state covariance P
		// P_priori = P1 + P2 = (A * P * A') + ( W * Q * W')
		// P1
		mat_transpose(6, 6, kalman_states[i].A, A_t);
		mat_mul(6, 6, kalman_states[i].A, 6, 6, kalman_states[i].P, P1);
		mat_mul(6, 6, P1, 6, 6, A_t, P1);
		// P2
		mat_transpose(6, 2, kalman_states[i].W, W_t);
		mat_mul(6, 2, kalman_states[i].W, 2, 2, kalman_states[i].Q, P2_part);
		mat_mul(6, 2, P2_part, 2, 6, W_t, P2);
		// P_priori stored in P1
		mat_sum(6, 6, P1, P2, P1);

	//
	//---------------------------------------------------------------------

	//---------------------------------------------------------------------
	// A POSTERIORI STAGE
	//---------------------------------------------------------------------
	//
		// evaluate Kalman gain K
		// K = K1 * inverse(K2) = (P_new * C') * inverse(C * P_new * C' + R);
		// K1
		mat_transpose(3, 6, kalman_states[i].C, C_t);
		mat_mul(6, 6, P1, 6, 3, C_t, K1);
		// K2
		mat_mul(3, 6, kalman_states[i].C, 6, 6, P1, K2_part);
		mat_mul(3, 6, K2_part, 6, 3, C_t, K2);
	   	mat_sum(3, 3, K2, kalman_states[i].R, K2);
		// inverse(K2) stored in K2
		mat_3_inv(K2, K2);
		// K stored in K1
		mat_mul(6, 3, K1, 3, 3, K2, K1);

		// evaluate weighted innovation K * (measures - C * state_priori)
		innovation[0][0] = x_m - state_priori.x;
		innovation[1][0] = y_m - state_priori.y;
		innovation[2][0] = theta_m - state_priori.theta;
		mat_mul(6, 3, K1, 3, 1, innovation, weigthed_innovation);

		// evaluate estimate as state_priori + weighted_innovation
		estimate->x = state_priori.x + weigthed_innovation[0][0];
		estimate->vx = state_priori.vx + weigthed_innovation[1][0];
		estimate->y = state_priori.y + weigthed_innovation[2][0];
		estimate->vy = state_priori.vy + weigthed_innovation[3][0];
		estimate->theta = state_priori.theta + weigthed_innovation[4][0];
		estimate->vtheta = state_priori.vtheta + weigthed_innovation[5][0];

		// update global error state covariance P
		// P_posteriori = P_priori - (K * C * P_priori)
		mat_mul(6, 3, K1, 3, 6, kalman_states[i].C, P2);
		mat_mul(6, 6, P2, 6, 6, P1, P2);
		mat_sub(6, 6, P1, P2, kalman_states[i].P);

	//
	//---------------------------------------------------------------------
}

//------------------------------------------------------------------------------
//	TASK CODE
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	REGULATOR TASK
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function set_initial_conditions
//	set the initial conditions for the simulation
//------------------------------------------------------------------------------
void	set_initial_conditions()
{
int		i;
float	x0, y0;

		for (i = 0; i < MAX_QUADROTORS; i++) {
			// Init state
			x0 = get_uniform_generic(L, WORLD_W - L);
			y0 = get_uniform_generic(L, WORLD_H - L);
			init_state(i, x0, 0, y0, 0, 0, 0);

			// Init estimate
			init_state_estimate(i, x0, 0, y0, 0, 0, 0);

			// Init trajectory
			traj_states[i].current_time = 0;
			traj_states[i].final_time = 3;
			traj_states[i].x0 = x0;
			traj_states[i].y0 = y0;
			traj_states[i].xf = x0;
			traj_states[i].yf = y0;
		}
}

//------------------------------------------------------------------------------
//	Function regulator_task
//------------------------------------------------------------------------------
void*	regulator_task(void* arg)
{
struct task_par* tp;
float	measure[3], real_input[2], reference_input[2];
float	period;
state	reference_trajectory, estimate, state;

		tp = (struct task_par*)arg;
		period = ((float)tp->period) / 1000;

		init_timespecs(tp);

		init_ekf_matrix(tp->id, period);

		while(1) {
			set_start_time(tp);

			// Recover the last evaluated state
			copy_state(&states[tp->id], &state, &dynamics_mutex[tp->id]);

			// Get reference trajectory and reference input
			get_reference_state(tp->id, period, &reference_trajectory, reference_input);
			update_trajectory_time(tp->id, period);

			// Evaluate the real input to the system
			feedback_control(kalman_states[tp->id].estimate, reference_trajectory,\
							 reference_input[0], reference_input[1], real_input);

			// Evaluate the output of the system
			system_io(tp->id, period, real_input[0], real_input[1], &state, measure);

			// Estimate the state
			ekf(tp->id, period, real_input[0], real_input[1], measure[0], measure[1], measure[2],
				&estimate);

			// Store inside global variables
			copy_state(&estimate, &(kalman_states[tp->id].estimate), &kalman_mutex[tp->id]);
			copy_state(&state, &states[tp->id], &dynamics_mutex[tp->id]);
			copy_state(&reference_trajectory, &desired_trajectories[tp->id], &desired_traj_mutex[tp->id]);
			copy_forces(real_input, &forces[tp->id], &force_mutex[tp->id]);

			thread_loop_end(tp);
		}
}


//------------------------------------------------------------------------------
//  GUIDANCE TASK
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function guidance_task
//------------------------------------------------------------------------------
void*	guidance_task(void* arg)
{
struct task_par* tp;
struct trajectory_state traj_p;

		tp = (struct task_par*)arg;

		init_timespecs(tp);

		while(1) {

			set_start_time(tp);

			if (get_guidance_state(tp->id)) {

				copy_traj_param(&traj_states[tp->id], &traj_p, &guidance_mutex[tp->id]);

				if (traj_p.current_time > traj_p.final_time + GUID_WAIT_TIME)
					set_traj_params(tp->id, get_uniform_generic(L, WORLD_W - L),\
									get_uniform_generic(L, WORLD_H - L));
			}

			thread_loop_end(tp);
		}
}

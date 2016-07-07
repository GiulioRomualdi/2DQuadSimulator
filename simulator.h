#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <pthread.h>

//------------------------------------------------------------------------------
//	2D QUADCOPTER CONSTANTS
//------------------------------------------------------------------------------
#define	L					0.38			// quadcopter length in m
#define	M					1.0				// quadcopter mass in Kg
#define IZ					(M * L * L /12)	// moment of inertia around axis x
#define G					9.81			// gravitational acceleration in m/s^2
#define MAX_QUADROTORS		5	   			// max number of quadrotor
//------------------------------------------------------------------------------
//	FLYING AREA CONSTANTS
//------------------------------------------------------------------------------
#define	WORLD_W				8				// horizontal bound in m
#define WORLD_H				4				// vertical bound in m
//------------------------------------------------------------------------------
//	TASK CONSTANTS
//------------------------------------------------------------------------------
#define GUIDANCE_PERIOD		1000.0			// guidance task period in ms
#define GUIDANCE_DEADLINE	1000.0			// guidance task relative deadline in ms
#define REGULATOR_PERIOD	10.0			// regulator task period in ms
#define REGULATOR_DEADLINE	10.0			// regulator task relative deadline in ms

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	state {							// quadrotor state
	   	float			x;				// x coordinate (m)
		float			y;				// y coordinate (m)
		float			theta;			// pitch angle (rad)
		float			vx;		   		// horizontal velocity (m/s)
		float			vy;				// vertical velocity (m/s)
		float			vtheta;			// pitch rate (rad/s)
};
typedef struct state state;

struct	kalman_state {
		struct state	estimate;		// state estimate
		float 			P[6][6];		// error state covariance matrix
		float			Q[2][2];		// process noise covariance matrix
		float			R[3][3];		// measurement noise covariance matrix
		float			A[6][6];		// state Jacobian
		float			W[6][2];		// noise input Jacobian
		float			C[3][6];		// output Jacobian
};

struct	trajectory_state {
		float			current_time;	// in seconds
		float			final_time;		// in seconds
		float			x0;				// in m
		float			y0;				// in m
		float			xf;				// in m
		float			yf;				// in m
};

struct	force {
		float			force_left;		// left force in N
		float			force_right;	// right force in N
};

//------------------------------------------------------------------------------
//	GLOBAL VARIABLE EXTERN DECLARATIONS
//------------------------------------------------------------------------------
extern struct state states[MAX_QUADROTORS];
extern struct state	desired_trajectories[MAX_QUADROTORS];
extern struct kalman_state kalman_states[MAX_QUADROTORS];
extern struct trajectory_state traj_states[MAX_QUADROTORS];
extern struct force forces[MAX_QUADROTORS];
extern int guid_switches[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable with type 'state'
//------------------------------------------------------------------------------
void 	copy_state(state *src, state *dst, pthread_mutex_t *mutex);

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable with type 'trajectory_state'
//------------------------------------------------------------------------------
void	copy_traj_param(struct trajectory_state *src,\
						struct trajectory_state *dst,\
						pthread_mutex_t *mutex);

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable 'guid_switch'
//------------------------------------------------------------------------------
void	init_guidance_switches();
int		get_guidance_state(int index);
void	switch_guidance(int index);

//------------------------------------------------------------------------------
//	THREAD CODE FUNCTIONS
//------------------------------------------------------------------------------
void	set_initial_conditions();
void*	regulator_task(void* arg);
void*	guidance_task(void* arg);

#endif

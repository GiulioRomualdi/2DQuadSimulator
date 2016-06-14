#ifndef SIMULATOR_H
#define SIMULATOR_H

//------------------------------------------------------------------------------
//	2D QUADCOPTER CONSTANTS
//------------------------------------------------------------------------------
#define MAX_QUADROTORS	5			// max number of quadrotor

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	state {						// quadrotor state
	   	float			x;			// x coordinate (m)
		float			y;			// y coordinate (m)
		float			theta;		// pitch angle (rad)
		float			vx;		   	// horizontal velocity (m/s)
		float			vy;			// vertical velocity (m/s)
		float			vtheta;		// pitch rate (rad/s)
};

struct	kalman_state {
		struct state	estimate;	// state estimate
		float 			P0[6][6];	// initial state covariance matrix
		float 			P[6][6];	// error state covariance matrix
		float			Q[2][2];	// process noise covariance matrix
		float			R[3][3];	// measurement noise covariance matrix
		float			A[6][6];	// state Jacobian
		float			W[6][2];	// noise input Jacobian
		float			C[3][6];	// output Jacobian
};

//------------------------------------------------------------------------------
//	GLOABAL DATA STRUCTURES
//------------------------------------------------------------------------------
extern struct state quads[MAX_QUADROTORS];
extern struct kalman_state kalman_states[MAX_QUADROTORS];

#endif

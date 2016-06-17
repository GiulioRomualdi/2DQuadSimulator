#ifndef SIMULATOR_H
#define SIMULATOR_H

//------------------------------------------------------------------------------
//	2D QUADCOPTER CONSTANTS
//------------------------------------------------------------------------------
#define MAX_QUADROTORS	5					// max number of quadrotor

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
typedef struct state state;

struct	kalman_state {
		struct state	estimate;	// state estimate
		float 			P[6][6];	// error state covariance matrix
		float			Q[2][2];	// process noise covariance matrix
		float			R[3][3];	// measurement noise covariance matrix
		float			A[6][6];	// state Jacobian
		float			W[6][2];	// noise input Jacobian
		float			C[3][6];	// output Jacobian
};

//------------------------------------------------------------------------------
//	GLOBAL VARIABLE EXTERN DECLARATIONS
//------------------------------------------------------------------------------
extern struct state states[MAX_QUADROTORS];
extern struct kalman_state kalman_states[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	EXTENDED KALMAN FILTER
//------------------------------------------------------------------------------
void 	init_A(int i, float T);
void 	update_A(int i, float T, float a_priori_theta, float fl, float fr);
void 	init_W(int i, float T);
void 	init_C(int i);
void 	init_P(int i, float sigma_x, float sigma_vx,\
				float sigma_y, float sigma_vy,\
				float sigma_theta, float sigma_vtheta);
void 	init_Q(int i, float sigma_wind_x, float sigma_wind_y);
void 	init_R(int i, float sigma_x, float sigma_y, float sigma_theta);
void 	init_state_estimate(int i, float x, float vx, float y,\
							float vy, float theta, float vtheta);
void	ekf(int i, float T, float fl, float fr, float x_m,\
			float y_m, float theta_m, state* estimate);
void	simulate();
#endif

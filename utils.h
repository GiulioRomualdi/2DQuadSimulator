#ifndef UTILS_H
#define UTILS_H

#include <time.h>
#include <pthread.h>
#include "simulator.h"

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	task_par {								// task parameters
		char				task_name[20];		// task name
		int					id;					// task id
		int					period;				// in milliseconds
		int					deadline;			// relative in milliseconds
		int					priority;			// in [0, 99]
		int 				dmiss;				// no. of misses
		struct	timespec 	activation_time;	// next activation time
		struct	timespec	abs_deadline;		// absolute
		struct	timespec 	start_time;			// task current start time
		struct	timespec	finish_time;		// task current finish time
		struct	timespec	wcet;				// worst case execution time
		pthread_mutex_t		mutex;				// mutex
};
typedef struct task_par task_par;

//------------------------------------------------------------------------------
//	GLOBAL VARIABLE EXTERN DECLARATIONS
//------------------------------------------------------------------------------
extern pthread_mutex_t guidance_mutex[MAX_QUADROTORS];
extern pthread_mutex_t dynamics_mutex[MAX_QUADROTORS];
extern pthread_mutex_t kalman_mutex[MAX_QUADROTORS];
extern pthread_mutex_t force_mutex[MAX_QUADROTORS];
extern pthread_mutex_t desired_traj_mutex[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	MATRIX HANDLING FUNCTION
//------------------------------------------------------------------------------
void	mat_zero(int n, int m, float matrix[][m]);
void	mat_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void 	mat_sub(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void	mat_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   	    float dst[op1_n][op2_m]);
void 	mat_transpose(int n, int m, float src[][m], float dst[][n]);
void	mat_3_inv(float matrix[3][3], float dst[3][3]);
void	rotate(float matrix[3][3], float angle);
void	translate(float matrix[3][3], float x, float y);
void	scale(float matrix[3][3], float k);

//------------------------------------------------------------------------------
//	RANDOM NUMBERS GENERATION
//------------------------------------------------------------------------------
void	init_random_generator();
float	get_uniform(float T);
float	get_uniform_generic(float a, float b);
float	get_gaussian(float std);

//-----------------------------------------------------------------------------
//	TIMESPEC HANDLING
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//	THREAD MANAGEMENT
//-----------------------------------------------------------------------------
void	init_timespecs(task_par* tp);
void	set_start_time(task_par* tp);
void	set_finish_time(task_par* tp);
void	zero_wcet(task_par* tp);
void	update_activation_time(task_par* tp);
void	update_abs_deadline(task_par* tp);
void	update_wcet(task_par* tp);
void	wait_for_period(task_par* tp);
int		deadline_miss(task_par* tp);
void	mutex_init();

#endif

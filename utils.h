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

//------------------------------------------------------------------------------
//	TASK REGULATOR
//------------------------------------------------------------------------------
extern task_par 		regulator_tp[MAX_QUADROTORS];
extern pthread_t 		regulator_tid[MAX_QUADROTORS];
extern pthread_attr_t 	regulator_attr[MAX_QUADROTORS];
//------------------------------------------------------------------------------
//	TASK GUIDANCE
//------------------------------------------------------------------------------
extern task_par 		guidance_tp[MAX_QUADROTORS];
extern pthread_t		guidance_tid[MAX_QUADROTORS];
extern pthread_attr_t	guidance_attr[MAX_QUADROTORS];
//------------------------------------------------------------------------------
//	TASK GUI
//------------------------------------------------------------------------------
extern task_par			gui_tp[1];
extern pthread_t		gui_tid[1];
extern pthread_attr_t	gui_attr[1];
//------------------------------------------------------------------------------
//	TASK USER
//------------------------------------------------------------------------------
extern task_par			user_tp[1];
extern pthread_t		user_tid[1];
extern pthread_attr_t	user_attr[1];
//------------------------------------------------------------------------------
//	MUTEX
//------------------------------------------------------------------------------
extern pthread_mutex_t 	guidance_mutex[MAX_QUADROTORS];
extern pthread_mutex_t 	dynamics_mutex[MAX_QUADROTORS];
extern pthread_mutex_t 	kalman_mutex[MAX_QUADROTORS];
extern pthread_mutex_t 	force_mutex[MAX_QUADROTORS];
extern pthread_mutex_t 	desired_traj_mutex[MAX_QUADROTORS];
extern pthread_mutex_t	guid_switches_mutex[MAX_QUADROTORS];
extern pthread_mutex_t	selected_quad_mutex;

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
//	MATH FUNCTIONS
//-----------------------------------------------------------------------------
int		modulo(int a, int b);

//-----------------------------------------------------------------------------
//	TIMESPEC HANDLING
//-----------------------------------------------------------------------------
float	time_to_ms(struct timespec* time);

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
void	aperiodic_wait(struct task_par* tp);
#endif

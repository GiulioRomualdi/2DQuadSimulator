//------------------------------------------------------------------------------
//	MAIN.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <pthread.h>
#include <stdio.h>
#include <error.h>
#include "utils.h"
#include "simulator.h"
#include "gui.h"

//------------------------------------------------------------------------------
//	TASK CONSTANTS
//------------------------------------------------------------------------------
#define GUIDANCE_PERIOD		1000	// guidance task period in ms
#define GUIDANCE_DEADLINE	1000	// guidance task relative deadline in ms
#define REGULATOR_PERIOD	10		// regulator task period in ms
#define REGULATOR_DEADLINE	10		// regulator task relative deadline in ms
#define GUI_PERIOD			34		// gui task period in ms
#define GUI_DEADLINE		34		// gui task relative deadline in ms

//------------------------------------------------------------------------------
//	Function create_task
// 	creates a task given the task parameters, id, attribute, body,
//	period, relative deadline and the no. of replica required;
//	return 0 if success, one of the ERRNO otherwise
//------------------------------------------------------------------------------
static
int		create_task(struct task_par tp[], pthread_t tid[], pthread_attr_t attr[],\
					void*(*task_body)(void*), int period, int deadline, int replica)
{
int 	i;
int 	error;

		 for(i = 0; i < replica; i++) {
			tp[i].id = i;
			tp[i].period = period;
			tp[i].deadline = deadline;
			tp[i].dmiss = 0;

			error = pthread_attr_init(&attr[i]);
			if (error != 0) {
				perror("(attr_init) Error:");
				return error;
			}

		    /* error = pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED); */
			/* printf("%d/n", error); */
			/* if (error != 0) { */
			/* 	perror("(attr_setinheritsched) Error:"); */
			/* 	return error; */
			/* } */
			/* error = pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO); */
			/* printf("%d/n", error); */
			/* if (error != 0) { */
			/* 	perror("(attr_setschedpolicy) Error:"); */
			/* 	return error; */
			/* } */

			error = pthread_create(&tid[i], &attr[i], task_body, &tp[i]);
			if (error != 0) {
				perror("(pthread_create) Error:");
				return error;
			}
		}
		return 0;
}

//------------------------------------------------------------------------------
//	Function set_initial_condition
//	set the initial condition for the dynamics and for kalman filter
//------------------------------------------------------------------------------
static
void	set_initial_condition()
{
int		i;
float	x0, y0;

		for(i = 0; i < MAX_QUADROTORS; i++) {
			// Init state dynamics and ek
			/* x0 = get_uniform(WORLD_W); */
			/* y0 = get_uniform(WORLD_H); */
			x0 = 1;
			y0 = 1;
			init_state(i, x0, 0, y0, 0, 0, 0);
			init_state_estimate(i, x0, 0, y0, 0, 0, 0);

			// Set initial the current and final time of the trajectory
			traj_states[i].current_time = 0;
			traj_states[i].final_time = 3;
			traj_states[i].x0 = x0;
			traj_states[i].y0 = y0;
			traj_states[i].xf = x0;
			traj_states[i].yf = y0;
		}
}

int				main()
{
int				error, i;

struct task_par regulator_tp[MAX_QUADROTORS];
pthread_t		regulator_tid[MAX_QUADROTORS];
pthread_attr_t	regulator_attr[MAX_QUADROTORS];

struct task_par guidance_tp[MAX_QUADROTORS];
pthread_t		guidance_tid[MAX_QUADROTORS];
pthread_attr_t	guidance_attr[MAX_QUADROTORS];

struct task_par	gui_tp[1];
pthread_t		gui_tid[1];
pthread_attr_t	gui_attr[1];

				// Init
				init_random_generator();
				mutex_init();
				set_initial_condition();

				// Create tasks
				error = create_task(guidance_tp, guidance_tid, guidance_attr,\
									guidance_task, GUIDANCE_PERIOD,\
									GUIDANCE_DEADLINE, MAX_QUADROTORS);
				if(error != 0)
						return error;


				error = create_task(regulator_tp, regulator_tid, regulator_attr,\
									regulator_task, REGULATOR_PERIOD,\
									REGULATOR_DEADLINE, MAX_QUADROTORS);
				if(error != 0)
						return error;

				error = create_task(gui_tp, gui_tid, gui_attr,\
									gui_task, GUI_PERIOD,\
									GUI_DEADLINE, 1);
				if(error != 0)
						return error;

				for(i = 0; i < MAX_QUADROTORS; i++) {
					pthread_join(regulator_tid[i], NULL);
					pthread_join(guidance_tid[i], NULL);
				}
				pthread_join(gui_tid[0], NULL);

				return 0;
}

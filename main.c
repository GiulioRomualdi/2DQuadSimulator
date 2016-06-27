//------------------------------------------------------------------------------
//	MAIN.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <pthread.h>
#include <stdio.h>
#include "utils.h"
#include "simulator.h"

int				main()
{
struct task_par regulator_tp[MAX_QUADROTORS];
struct task_par guidance_tp[MAX_QUADROTORS];
pthread_t		regulator_tid[MAX_QUADROTORS];
pthread_t		guidance_tid[MAX_QUADROTORS];
pthread_attr_t	regulator_attr[MAX_QUADROTORS];
pthread_attr_t	guidance_attr[MAX_QUADROTORS];
int	i;

	i = 0;

	// Init required for gaussian noise generation (once for all threads)
	init_random_generator();

	// Init mutex
	mutex_init();

	// Init state dynamics and ekf
	init_state(i, 1, 0, 1, 0, 0, 0);
	init_state_estimate(i, 1, 0, 1, 0, 0, 0);

	// Set initial the current and final time of the trajectory
	traj_states[i].current_time = 0;
	traj_states[i].final_time = 3;
	traj_states[i].x0 = 1;
	traj_states[i].y0 = 1;
	traj_states[i].xf = 1;
	traj_states[i].yf = 1;

	// Thread creation
	// Guidance
	guidance_tp[i].id = i;
	guidance_tp[i].period = 1000;
	guidance_tp[i].deadline = 1000;
	guidance_tp[i].dmiss = 0;

	pthread_attr_init(&guidance_attr[i]);
	/* pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED); */
	/* pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO); */

	pthread_create(&guidance_tid[i], &guidance_attr[i], guidance_task, &guidance_tp[i]);

	// Regulator
	regulator_tp[i].id = i;
	regulator_tp[i].period = 1000;
	regulator_tp[i].deadline = 1000;
	regulator_tp[i].dmiss = 0;

	pthread_attr_init(&regulator_attr[i]);
	/* pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED); */
	/* pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO); */

	pthread_create(&regulator_tid[i], &regulator_attr[i], regulator_task, &regulator_tp[i]);

	pthread_join(regulator_tid[i], NULL);
	pthread_join(guidance_tid[i], NULL);

	return 0;
}

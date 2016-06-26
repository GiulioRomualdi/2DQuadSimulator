//------------------------------------------------------------------------------
//	MAIN.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <pthread.h>
#include <stdio.h>
#include "utils.h"
#include "simulator.h"

int				main()
{
struct task_par tp[MAX_QUADROTORS];
pthread_t		tid[MAX_QUADROTORS];
pthread_attr_t	attr[MAX_QUADROTORS];
int	i;

	// Init required for gaussian noise generation (once for all threads)
	init_random_generator();

	// Init mutex
	mutex_init();

	// Thread creation
	i = 0;

	tp[i].id = i;
	tp[i].period = 1;
	tp[i].deadline = 1;
	tp[i].dmiss = 0;

	pthread_attr_init(&attr[i]);
	/* pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED); */
	/* pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO); */

	pthread_create(&tid[i], &attr[i], regulator_task, &tp[i]);

	pthread_join(tid[i], NULL);
}

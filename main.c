//------------------------------------------------------------------------------
//	MAIN.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <pthread.h>
#include <error.h>
#include <stdlib.h>
#include <stdio.h>
#include <allegro.h>
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
#define GUI_PERIOD			15		// gui task period in ms
#define GUI_DEADLINE		15		// gui task relative deadline in ms

//------------------------------------------------------------------------------
//	GLOABAL VARIABLE DEFINITIONS
//------------------------------------------------------------------------------
task_par 		regulator_tp[MAX_QUADROTORS];
pthread_t 		regulator_tid[MAX_QUADROTORS];
pthread_attr_t 	regulator_attr[MAX_QUADROTORS];
task_par 		guidance_tp[MAX_QUADROTORS];
pthread_t		guidance_tid[MAX_QUADROTORS];
pthread_attr_t	guidance_attr[MAX_QUADROTORS];
task_par		gui_tp[1];
pthread_t		gui_tid[1];
pthread_attr_t	gui_attr[1];

//------------------------------------------------------------------------------
//	Function create_task
// 	creates a task given the task parameters, id, attribute, body,
//	period, relative deadline and the no. of replica required;
//	return 0 if success, one of the ERRNO otherwise
//------------------------------------------------------------------------------
static
int		create_task(task_par tp[], pthread_t tid[], pthread_attr_t attr[],\
					void*(*task_body)(void*), int period, int deadline, int replica)
{
int		i;
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

			error = pthread_create(&tid[i], &attr[i], task_body, &tp[i]);
			if (error != 0) {
				perror("(pthread_create) Error:");
				return error;
			}
		}
		return 0;
}

int		wait_for_task_end()
{
int 	i, error;

		for(i = 0; i < MAX_QUADROTORS; i++) {
			error = pthread_join(regulator_tid[i], NULL);
			if (error != 0) {
				perror("(pthread_join) Error:");
				return error;
			}

			error = pthread_join(guidance_tid[i], NULL);
			if (error != 0) {
				perror("(pthread_join) Error:");
				return error;
			}
		}
		error = pthread_join(gui_tid[0], NULL);
		if (error != 0) {
			perror("(pthread_join) Error:");
		}

		return 0;
}

//------------------------------------------------------------------------------
//	Function init
//------------------------------------------------------------------------------
static
int		init()
{
int		i, error;

		// allegro
		allegro_init();
		set_color_depth(32);
		set_gfx_mode(GFX_AUTODETECT_WINDOWED, 1024, 576, 0, 0);

		// random number generation
		init_random_generator();

		// mutexes
		mutex_init();

		// dynamics, ekf and trajectories initial conditions
		for (i = 0; i < MAX_QUADROTORS; i++)
			set_initial_condition(i);

		// threads
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
							gui_task, GUI_PERIOD, GUI_DEADLINE, 1);
		if(error != 0)
			return error;

		return 0;
}

int		main()
{
		if (init() != 0)
			return EXIT_FAILURE;

		if (wait_for_task_end() != 0)
			return EXIT_FAILURE;

		allegro_exit();

		return EXIT_SUCCESS;
}

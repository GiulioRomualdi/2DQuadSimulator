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
//	Function create_task
// 	creates a task given the task name, parameters, id, attribute, body,
//	period, relative deadline and the no. of replica required;
//	return 0 if success, one of the ERRNO otherwise
//------------------------------------------------------------------------------
static
int		create_task(char* task_name, task_par tp[], pthread_t tid[],\
					pthread_attr_t attr[], void*(*task_body)(void*),\
					int period, int deadline, int replica)
{
int		i;
int 	error;

		for(i = 0; i < replica; i++) {
			tp[i].id = i;
			tp[i].period = period;
			tp[i].deadline = deadline;
			tp[i].dmiss = 0;
			strcpy(tp[i].task_name, task_name);
			pthread_mutex_init(&tp[i].mutex, NULL);
			zero_wcet(&(tp[i]));

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

//------------------------------------------------------------------------------
//	Function wait_for_task_end
//------------------------------------------------------------------------------
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
		error = pthread_join(user_tid[0], NULL);
		if (error != 0) {
			perror("(pthread_user) Error:");
		}

		return 0;
}

//------------------------------------------------------------------------------
//	Function init
//------------------------------------------------------------------------------
static
int		init()
{
int	  	error;

		// allegro
		allegro_init();
		set_color_depth(32);
		set_gfx_mode(GFX_AUTODETECT_WINDOWED, WINDOW_W, WINDOW_H, 0, 0);
		install_keyboard();

		// init selected quadrotor
		init_selected_quad();

		// init guidance switches
		init_guidance_switches();

		// random number generation
		init_random_generator();

		// mutexes
		mutex_init();

		// dynamics, ekf and trajectories initial conditions
		set_initial_conditions();

		// threads
		error = create_task("Guidance", guidance_tp, guidance_tid,\
							guidance_attr, guidance_task, GUIDANCE_PERIOD,\
							GUIDANCE_DEADLINE, MAX_QUADROTORS);
		if(error != 0)
			return error;

		error = create_task("Controller", regulator_tp, regulator_tid,\
							regulator_attr,	regulator_task, REGULATOR_PERIOD,\
							REGULATOR_DEADLINE, MAX_QUADROTORS);
		if(error != 0)
			return error;

		error = create_task("Gui", gui_tp, gui_tid, gui_attr,\
							gui_task, GUI_PERIOD, GUI_DEADLINE, 1);
		if(error != 0)
			return error;

		error = create_task("User", user_tp, user_tid, user_attr,\
							user_task, GUI_PERIOD, 0, 1);
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

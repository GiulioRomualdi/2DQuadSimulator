//------------------------------------------------------------------------------
//	MAIN.C
//------------------------------------------------------------------------------

#include <allegro.h>
#include "utils.h"
#include "simulator.h"
#include "gui.h"

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
		install_mouse();
		enable_hardware_cursor();
		show_mouse(screen);

		init_selected_quad();
		init_target_selection_mode();
		init_guidance_switches();
		init_random_generator();
		mutex_init();
		set_initial_conditions();

		// threads
		error = create_task("Guidance", guidance_tp, guidance_tid,\
							guidance_task, GUIDANCE_PERIOD,\
							GUIDANCE_DEADLINE, MAX_QUADROTORS);
		if(error != 0)
			return error;

		error = create_task("Controller", regulator_tp, regulator_tid,\
							regulator_task, REGULATOR_PERIOD,\
							REGULATOR_DEADLINE, MAX_QUADROTORS);
		if(error != 0)
			return error;

		error = create_task("Gui", gui_tp, gui_tid,\
							gui_task, GUI_PERIOD, GUI_DEADLINE, 1);
		if(error != 0)
			return error;

		error = create_task("User", user_tp, user_tid,\
							user_task, GUI_PERIOD, 0, 1);
		if(error != 0)
			return error;

		return 0;
}

int		main()
{
		if (init() != 0)
			return EXIT_FAILURE;

		if (wait_for_tasks_end() != 0)
			return EXIT_FAILURE;

		mutex_destroy();

		allegro_exit();

		return EXIT_SUCCESS;
}

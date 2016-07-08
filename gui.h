#ifndef GUI_H
#define GUI_H

#include <pthread.h>

//------------------------------------------------------------------------------
//	TASK CONSTANTS
//------------------------------------------------------------------------------
#define GUI_PERIOD		34					// gui task period in ms
#define GUI_DEADLINE	34					// gui task relative deadline in ms
#define USER_PERIOD		500.0				// user task period in ms
//------------------------------------------------------------------------------
//	PLOT CONSTANTS
//------------------------------------------------------------------------------
#define MAX_PLOT_TIME	8					// maximum plot time in s
#define BUFF_SIZE		(MAX_PLOT_TIME * \
						 1000 / GUI_PERIOD)	// buffer size
//------------------------------------------------------------------------------
//	SIZE CONSTANTS
//------------------------------------------------------------------------------
#define WINDOW_W		1350				// main window width
#define WINDOW_H		750					// main window height

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	plot_data {
		float				buffer[BUFF_SIZE];	// data
		int					index;				// index of the oldest sample in buffer
		pthread_mutex_t		index_mutex;		// mutex for index
};
typedef struct plot_data plot_data;

struct	quad_coordinates {
		float				x_left;				// quadrotor left vertex (x)
		float				x_right;			// quadrotor right vertex (x)
		float				x_down;				// quadrotor down vertex (x)
		float				x_centre;			// quadrotor center (x)
		float				x_fl;				// quadrotor left force (x)
		float				x_fr;				// quadrotor left force (x)
		float				y_left;				// quadrotor left vertex (y)
		float				y_right;			// quadrotor right vertex (y)
		float				y_down;				// quadrotor down vertex (y)
		float				y_centre;			// quadrotor center (y)
		float				y_fl;				// quadrotor left force (y)
		float				y_fr;				// quadrotor left force (y)
};

//------------------------------------------------------------------------------
//	GLOBAL VARIABLE EXTERN DECLARATIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable 'selected_quadrotor'
//------------------------------------------------------------------------------
void	init_selected_quad();
void	init_target_selection_mode();

//------------------------------------------------------------------------------
//	THREAD CODE
//------------------------------------------------------------------------------
void*	gui_task(void* arg);
void*	user_task(void* arg);

#endif

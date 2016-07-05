#ifndef GUI_H
#define GUI_H

#include <pthread.h>

//------------------------------------------------------------------------------
//	TASK CONSTANTS
//------------------------------------------------------------------------------
#define GUI_PERIOD		34			// gui task period in ms
#define GUI_DEADLINE	34			// gui task relative deadline in ms
#define USER_PERIOD		500.0		// user task period in ms

//------------------------------------------------------------------------------
//	PLOT CONSTANTS
//------------------------------------------------------------------------------
#define MAX_PLOT_TIME	8			// maximum plot time in s
#define BUFF_SIZE		(MAX_PLOT_TIME * 1000 / GUI_PERIOD)
//------------------------------------------------------------------------------
//	SIZE CONSTANTS
//------------------------------------------------------------------------------
#define WINDOW_W		1350		// main window width
#define WINDOW_H		750			// main window height

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	plot_data {
		float				buffer[BUFF_SIZE];	// data
		int					index;				// index of the oldest sample in buffer
		pthread_mutex_t		index_mutex;		// mutex for index
};
typedef struct plot_data plot_data;

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

//------------------------------------------------------------------------------
//	THREAD CODE
//------------------------------------------------------------------------------
void*	gui_task(void* arg);
void*	user_task(void* arg);

#endif

#ifndef GUI_H
#define GUI_H

//------------------------------------------------------------------------------
//	TASK CONSTANTS
//------------------------------------------------------------------------------
#define GUI_PERIOD		34		// gui task period in ms
#define GUI_DEADLINE	34		// gui task relative deadline in ms

//------------------------------------------------------------------------------
//	PLOT CONSTANTS
//------------------------------------------------------------------------------
#define MAX_PLOT_TIME	8		// maximum plot time in s
#define BUFF_SIZE		MAX_PLOT_TIME * 1000 / GUI_PERIOD

//------------------------------------------------------------------------------
//	DATA STRUCTURES DECLARATIONS
//------------------------------------------------------------------------------
struct	plot_data {
		float	buffer[BUFF_SIZE];	// data
		int		index;				// index of the oldest sample in buffer
};
typedef struct plot_data plot_data;

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	THREAD CODE
//------------------------------------------------------------------------------
void*	gui_task(void* arg);

#endif

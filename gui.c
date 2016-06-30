//------------------------------------------------------------------------------
//	GUI.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <allegro.h>
#include <stdio.h>
#include "simulator.h"
#include "utils.h"
#include "gui.h"

//------------------------------------------------------------------------------
//	FLYING AREA CONSTANTS
//------------------------------------------------------------------------------
#define FLY_W			1024.0				// flying area width in pixel
#define FLY_H			576.0				// flying area heigth in pixel
#define	FLY_X			0					// flying area upper-left corner (x)
#define	FLY_Y			0					// flying area upper-left corner (y)
#define	FLY_SCALING		FLY_W / WORLD_W		// scale factor
//------------------------------------------------------------------------------
//	PLOT AREA CONSTANTS
//-----------------------------------------------------------------------------
#define PLOT_W			300.0				// plot area width in pixel
#define PLOT_H		   	200.0				// plot area heigth in pixel
#define	PLOT_SCALE_X	PLOT_W /(BUFF_SIZE)	// scale factor for the time axis
//-----------------------------------------------------------------------------
#define	PLOT_X_X		1024.0				// plot x area upper-left corner (x)
#define	PLOT_X_Y		0.0					// plot x area upper-left corner (y)
#define	PLOT_X_MAX		0.5					// max value of data
//------------------------------------------------------------------------------
#define	PLOT_Y_X		1024.0				// plot y area upper-left corner (x)
#define	PLOT_Y_Y		200.0				// plot y area upper-left corner (y
#define	PLOT_Y_MAX		0.5					// max value of data
//------------------------------------------------------------------------------
#define	PLOT_THETA_X	1024.0				// plot theta area upper-left corner (x)
#define	PLOT_THETA_Y	400.0				// plot theta area upper-left corner (y)
#define	PLOT_THETA_MAX	0.05				// max value of data
//------------------------------------------------------------------------------
//	QUADROTOR GRAPHIC CONSTANTS
//------------------------------------------------------------------------------
#define	QUAD_HEIGHT		0.07				// height of the quadcopter in m
#define	F_THS			M * G / 2.0			// force threshold in N
#define	MAX_F_ARR		0.26				// max length of arrow representing force in m
#define	FORCE_SCALE		-MAX_F_ARR /(F_THS) // scale factor for force representation
//------------------------------------------------------------------------------
//	COLOR CONSTANTS
//------------------------------------------------------------------------------
#define FILL_COL_R		48					// red channel for fill color
#define	FILL_COL_G		49					// green channel for fill color
#define FILL_COL_B		49					// blue channel for fill colo
//------------------------------------------------------------------------------
#define FRAME_COL_R		119					// red channel for frame color
#define	FRAME_COL_G		110					// green channel for frame color
#define FRAME_COL_B		111					// blue channel for frame color
//------------------------------------------------------------------------------
#define GROUND_COL_R	66					// red channel for frame color
#define	GROUND_COL_G	76					// green channel for frame color
#define GROUND_COL_B	68					// blue channel for frame color
//------------------------------------------------------------------------------
#define FORCE_COL_R		255					// red channel for frame color
#define FORCE_COL_G		0					// green channel for frame color
#define FORCE_COL_B		0					// blue channel for frame color
//------------------------------------------------------------------------------
#define PLOT_COL1_R		17					// red channel for plot signal # 1
#define PLOT_COL1_G		150					// green channel for plot signal # 1
#define PLOT_COL1_B		240					// blue channel for plot signal # 1
//------------------------------------------------------------------------------
#define PLOT_COL2_R		251					// red channel for plot signal # 2
#define PLOT_COL2_G		188					// green channel for plot signal # 2
#define PLOT_COL2_B		5					// blue channel for plot signal # 2
//------------------------------------------------------------------------------
#define PLOT_T_AXIS_R  	48					// red channel for plot time axis
#define PLOT_T_AXIS_G	49					// green channel for plot time axis
#define PLOT_T_AXIS_B	49					// blue channel for plot time axis

//------------------------------------------------------------------------------
//	DRAWING FUNCTION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function draw_quadrotor
//  draw a quadrotor given the coordinates 'x', 'y', 'theta'
//	and the forces 'fl', 'fr'
//------------------------------------------------------------------------------
static
void	draw_quadrotor(BITMAP* bitmap, float x, float y, float theta, float fl, float fr)
{
int		fill_color, frame_color, force_color;
float	x_left, x_right, x_down, x_centre, x_fl, x_fr;
float	y_left, y_right, y_down, y_centre, y_fl, y_fr;
float	rot[3][3], transl[3][3], scale_[3][3], transformation[3][3];
float 	coords[3][6] = {{-L,  L,  0,           0, -L,                L},
						{ 0,  0, -QUAD_HEIGHT, 0,  fl * FORCE_SCALE, fr * FORCE_SCALE},
						{ 1,  1,  1,           1,  1,                1}};

		// Transform body fixed coordinates into world coordinates
		// New coordinates are stored in 'coordinates'
		rotate(rot, theta);
		translate(transl, x, y);
		scale(scale_, FLY_SCALING);
		mat_mul(3, 3, scale_, 3, 3, transl, transformation);
		mat_mul(3, 3, transformation, 3, 3, rot, transformation);
		mat_mul(3, 3, transformation, 3, 6, coords, coords);

		x_left = coords[0][0];
		x_right = coords[0][1];
		x_down = coords[0][2];
		x_centre = coords[0][3];
		x_fl = coords[0][4];
		x_fr = coords[0][5];
		y_left = FLY_H - coords[1][0];
		y_right = FLY_H - coords[1][1];
		y_down = FLY_H - coords[1][2];
		y_centre = FLY_H - coords[1][3];
		y_fl = FLY_H - coords[1][4];
		y_fr = FLY_H - coords[1][5];

		// Set colors
		fill_color = makecol(FILL_COL_R, FILL_COL_G, FILL_COL_B);
		frame_color = makecol(FRAME_COL_R, FRAME_COL_G, FRAME_COL_B);
		force_color = makecol(FORCE_COL_R, FORCE_COL_G, FORCE_COL_B);

		// Draw the quadcopter
		// Body
		triangle(bitmap, x_left, y_left, x_right, y_right, x_down, y_down, fill_color);
		circle(bitmap, x_centre, y_centre, FLY_SCALING * QUAD_HEIGHT, frame_color);
		// Edges
		line(bitmap, x_left, y_left, x_right, y_right, frame_color);
		line(bitmap, x_down, y_down, x_right, y_right, frame_color);
		line(bitmap, x_left, y_left, x_down, y_down, frame_color);
		// Forces
		line(bitmap, x_left, y_left, x_fl, y_fl, force_color);
		line(bitmap, x_right, y_right, x_fr, y_fr, force_color);
}

//------------------------------------------------------------------------------
//	Function draw_quads
//	draw all quadrotors
//------------------------------------------------------------------------------
static
void	draw_quads(BITMAP* bitmap)
{
float	x, y, theta, force_left, force_right;
int		i;

		clear_to_color(bitmap, 0);
		for(i = 0; i < MAX_QUADROTORS; i++) {
			// Get quadrotor state
			pthread_mutex_lock(&dynamics_mutex[i]);
			x = states[i].x;
			y = states[i].y;
			theta = states[i].theta;
			pthread_mutex_unlock(&dynamics_mutex[i]);

			// Get quadrotor forces
			pthread_mutex_lock(&force_mutex[i]);
			force_left = forces[i].force_left;
			force_right = forces[i].force_right;
			pthread_mutex_unlock(&force_mutex[i]);

			// Draw the quadrotor
			draw_quadrotor(bitmap, x, y, theta, force_left, force_right);
		}

		// Transfer image from bitmap to screen
		blit(bitmap, screen, 0, 0, FLY_X, FLY_Y, FLY_W, FLY_H);
}

//------------------------------------------------------------------------------
//	PLOT FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function plot_add_sample
//  add a sample to the buffer of the plot_data 'plot'
//------------------------------------------------------------------------------
static
void	plot_add_sample(plot_data* plot, float sample)
{
int 	i;

		// Update the index pointing to the oldest sample
		if(plot->index > 0)
			(plot->index)--;

		// Shift the buffer to the left
		for(i = plot->index; i < BUFF_SIZE - 1; i++)
			plot->buffer[i] = plot->buffer[i + 1];
		// Add the new sample to the buffer
		plot->buffer[BUFF_SIZE - 1] = sample;
}

//------------------------------------------------------------------------------
//	Function plot_signal
//  plot the signal, stored in the plot_data 'plot', into the BITMAP 'bitmap'
//------------------------------------------------------------------------------
static
void	plot_signal(BITMAP* bitmap, plot_data* plot, float max_value,\
					   int color_r, int color_g, int color_b)
{
int		i, r, g ,b, color;
float	h, s, v;
float	actual_value, next_value;

		// Evaluate main color of the plot
		rgb_to_hsv(color_r, color_g, color_b, &h, &s, &v);

		// Draw y = 0 axis
		line(bitmap, 0, PLOT_H / 2, PLOT_W, PLOT_H / 2,\
			 makecol(PLOT_T_AXIS_R, PLOT_T_AXIS_G, PLOT_T_AXIS_B));

		// Plot the signal
		for(i = plot->index; i < BUFF_SIZE - 1; i++) {

			// Evaluate color with scaled value (hsv)
			hsv_to_rgb(h, s, v * (float)i / (BUFF_SIZE), &r, &g, &b);
			color = makecol(r, g, b);

			// Draw the horizontal line
			actual_value = (float)PLOT_H / 2 * (1 - plot->buffer[i] / max_value);
			line(bitmap, i * PLOT_SCALE_X, actual_value, (i + 1) * PLOT_SCALE_X,\
				 actual_value, color);

			// Draw the vertical line
			next_value = (float)PLOT_H / 2 * (1 - plot->buffer[i + 1] / max_value);
			line(bitmap, (i + 1) * PLOT_SCALE_X, actual_value, (i + 1) * PLOT_SCALE_X,\
				 next_value, color);
		}
}

//------------------------------------------------------------------------------
//	Function draw_figure
//  draws a figure containing two signals
//------------------------------------------------------------------------------
static
void	draw_figure(BITMAP* bitmap, plot_data* signal_1, plot_data* signal_2,\
					float max_value, int upper_left_x, int upper_left_y)
{
		clear_to_color(bitmap, 0);

		plot_signal(bitmap, signal_1, max_value, PLOT_COL1_R, PLOT_COL1_G, PLOT_COL1_B);
		plot_signal(bitmap, signal_2, max_value, PLOT_COL2_R, PLOT_COL2_G, PLOT_COL2_B);

		blit(bitmap, screen, 0, 0, upper_left_x, upper_left_y, PLOT_W, PLOT_H);
}

//------------------------------------------------------------------------------
//	Function draw_plot_area
//  draws the plot area containing three figures (estimation and tracking error
//	of x, y and theta for the i-th quadcopter)
//------------------------------------------------------------------------------
static
void	draw_plot_area(BITMAP* bitmap_x, BITMAP* bitmap_y, BITMAP* bitmap_theta,\
					   plot_data* plot_x_est, plot_data* plot_x_track,\
					   plot_data* plot_y_est, plot_data* plot_y_track,\
					   plot_data* plot_theta_est, plot_data* plot_theta_track, int i)
{
float	x, x_est, x_traj;
float	y, y_est, y_traj;
float	theta, theta_est, theta_traj;

	 	pthread_mutex_lock(&dynamics_mutex[i]);
		x = states[i].x;
		y = states[i].y;
		theta = states[i].theta;
		pthread_mutex_unlock(&dynamics_mutex[i]);

	 	pthread_mutex_lock(&kalman_mutex[i]);
		x_est = kalman_states[i].estimate.x;
		y_est = kalman_states[i].estimate.y;
		theta_est = kalman_states[i].estimate.theta;
		pthread_mutex_unlock(&kalman_mutex[i]);

		pthread_mutex_lock(&desired_traj_mutex[i]);
		x_traj = desired_trajectories[i].x;
		y_traj = desired_trajectories[i].y;
		theta_traj = desired_trajectories[i].theta;
		pthread_mutex_unlock(&desired_traj_mutex[i]);

		plot_add_sample(plot_x_est, x - x_est);
		plot_add_sample(plot_x_track, x - x_traj);

		plot_add_sample(plot_y_est, y - y_est);
		plot_add_sample(plot_y_track, y - y_traj);

		plot_add_sample(plot_theta_est, theta - theta_est);
		plot_add_sample(plot_theta_track, theta - theta_traj);

		draw_figure(bitmap_x, plot_x_est, plot_x_track, PLOT_X_MAX,\
					PLOT_X_X, PLOT_X_Y);
		draw_figure(bitmap_y, plot_y_est, plot_y_track, PLOT_Y_MAX,\
					PLOT_Y_X, PLOT_Y_Y);
		draw_figure(bitmap_theta, plot_theta_est, plot_theta_track, PLOT_THETA_MAX,\
					PLOT_THETA_X, PLOT_THETA_Y);

}

//------------------------------------------------------------------------------
//	Function init_plot_data
//------------------------------------------------------------------------------
static
void	init_plot_data(plot_data* plot)
{
		plot->index = BUFF_SIZE;
}

//------------------------------------------------------------------------------
//	TASK CODE
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function gui_task
//------------------------------------------------------------------------------
void*		gui_task(void* arg)
{
task_par*	tp;
BITMAP		*fly_bitmap, *plot_x_bitmap, *plot_y_bitmap, *plot_theta_bitmap;
plot_data	plot_x_est, plot_x_track, plot_y_est, plot_y_track, plot_theta_est, plot_theta_track;
int			i;

			i = 0;
			fly_bitmap = create_bitmap(FLY_W, FLY_H);
			plot_x_bitmap = create_bitmap(PLOT_W, PLOT_H);
			plot_y_bitmap = create_bitmap(PLOT_W, PLOT_H);
			plot_theta_bitmap = create_bitmap(PLOT_W, PLOT_H);

			tp = (struct task_par*)arg;

			init_timespecs(tp);

			init_plot_data(&plot_x_est);
			init_plot_data(&plot_y_est);
			init_plot_data(&plot_theta_est);
			init_plot_data(&plot_x_track);
			init_plot_data(&plot_y_track);
			init_plot_data(&plot_theta_track);

			while(1) {
				// Draw quadrotors
				draw_quads(fly_bitmap);

				// Draw plots
				draw_plot_area(plot_x_bitmap, plot_y_bitmap, plot_theta_bitmap,\
							   &plot_x_est, &plot_x_track,\
							   &plot_y_est, &plot_y_track,\
							   &plot_theta_est, &plot_theta_track, i);

				// Handle thread parameters
				if(deadline_miss(tp))
					printf("%d\n", tp->dmiss);
				wait_for_period(tp);
				update_activation_time(tp);
				update_abs_deadline(tp);
			}
}

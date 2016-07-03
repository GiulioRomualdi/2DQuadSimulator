//------------------------------------------------------------------------------
//	GUI.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <allegro.h>
#include <stdio.h>
#include <math.h>
#include "simulator.h"
#include "utils.h"
#include "gui.h"

//------------------------------------------------------------------------------
// LAYOUT CONSTANTS
//------------------------------------------------------------------------------
#define MARGIN			30.0					// margin width in pixel
#define	LINE_OFFSET		30.0					// layout line offset in pixel
//------------------------------------------------------------------------------
//	FLYING AREA CONSTANTS
//------------------------------------------------------------------------------
#define FLY_W			960.0					// flying area width in pixel
#define FLY_H			540.0					// flying area heigth in pixel
#define	FLY_X			30.0					// flying area upper-left corner (x)
#define	FLY_Y			45.0					// flying area upper-left corner (y)
#define	FLY_SCALING		(FLY_W / WORLD_W)		// scale factor
//------------------------------------------------------------------------------
//	PLOT AREA CONSTANTS
//-----------------------------------------------------------------------------
#define	PLOT_TOP_MARGIN	FLY_Y					// top margin of plot area in pixel	
#define	PLOT_MARGIN		1.0						// inner plot margin in pixel
#define	PLOT_SPACING	12.0					// spacing between consecutive plots in pixel
#define	PLOT_TICK_WIDTH	30						// width occupied by text of ticks
//-----------------------------------------------------------------------------
#define PLOT_W			240.0					// plot area width in pixel
#define PLOT_H		   	170.0					// plot area heigth in pixel
#define	PLOT_W_SCALE	(PLOT_W / BUFF_SIZE)	// scale factor along the width
//-----------------------------------------------------------------------------
#define	PLOT_X_X		FLY_X + FLY_W + 2 * MARGIN + PLOT_TICK_WIDTH +  PLOT_MARGIN
#define	PLOT_X_Y		PLOT_TOP_MARGIN + PLOT_MARGIN
#define	PLOT_X_MAX		0.5						// max value of data
#define	PLOT_X_TICK		0.2						// axis tick in meters
#define	PLOT_X_SCALE	((PLOT_H / 2) / (PLOT_X_MAX))
//------------------------------------------------------------------------------
#define	PLOT_Y_X		PLOT_X_X
#define	PLOT_Y_Y		PLOT_X_Y + PLOT_H + 3 * PLOT_MARGIN + PLOT_SPACING
#define	PLOT_Y_MAX		0.5						// max value of data
#define	PLOT_Y_TICK		0.2						// axis tick in meters
#define	PLOT_Y_SCALE	((PLOT_H / 2) / (PLOT_Y_MAX))
//------------------------------------------------------------------------------
#define	PLOT_TH_X		PLOT_X_X
#define	PLOT_TH_Y		PLOT_Y_Y + PLOT_H + 3 * PLOT_MARGIN + PLOT_SPACING
#define	PLOT_TH_MAX		0.1						// max value of data
#define	PLOT_TH_TICK	0.04					// axis tick in degrees
#define	PLOT_TH_SCALE	((PLOT_H / 2) / (PLOT_TH_MAX))
//------------------------------------------------------------------------------
#define	PLOT_TIME_TICK	2						// time axis tick in seconds
#define PLOT_TIME_SCALE	((PLOT_W) / (MAX_PLOT_TIME))
//------------------------------------------------------------------------------
//	PERFORMANCE LOG AREA
//------------------------------------------------------------------------------
#define LOG_W			960.0					// log area width in pixel
#define LOG_H			100.0					// log area heigth in pixel
#define	LOG_X			30.0					// log area upper-left corner (x)
#define	LOG_Y			FLY_Y + FLY_H + 2 * MARGIN + 5	// log area upper-left corner (y)
#define	LOG_SPACING		160						// spacing between log widgets
//------------------------------------------------------------------------------
//	GUIDANCE LOG AREA
//------------------------------------------------------------------------------
#define GUIDANCE_W		270.0					// guidance area width in pixel
#define GUIDANCE_H		100.0					// guidance area heigth in pixel
#define	GUIDANCE_X		FLY_X + FLY_W + 2 * MARGIN
#define	GUIDANCE_Y		FLY_Y + FLY_H + 2 * MARGIN + 5
//------------------------------------------------------------------------------
//	QUADROTOR GRAPHIC CONSTANTS
//------------------------------------------------------------------------------
#define	QUAD_HEIGHT		0.07					// height of the quadcopter in m
#define	F_THS			(M * G / 2.0)			// force threshold in N
#define	MAX_F_ARR		0.26					// max length of arrow representing force in m
#define	FORCE_SCALE		(- MAX_F_ARR / F_THS) 	// scale factor for force representation
//------------------------------------------------------------------------------
//	COLOR CONSTANTS
//------------------------------------------------------------------------------
#define BG_COL_R		0						// red channel for background color
#define	BG_COL_G		0						// green channel for background color
#define BG_COL_B		0						// blue channel for background color
//------------------------------------------------------------------------------
#define LAYOUT_COL_R  	60						// red channel for layout
#define LAYOUT_COL_G	60						// green channel for layout
#define LAYOUT_COL_B	60						// blue channel for layout
//------------------------------------------------------------------------------
#define SKY_COL_R		0						// red channel for sky color
#define	SKY_COL_G	   	0						// green channel for sky color
#define SKY_COL_B		0						// blue channel for sky color
//------------------------------------------------------------------------------
#define TEXT_COL_R  	180						// red channel for text
#define TEXT_COL_G		180						// green channel for text
#define TEXT_COL_B		180						// blue channel for text
//------------------------------------------------------------------------------
// QUADCOPTER COLOR CONSTANTS
//------------------------------------------------------------------------------
#define FRAME_COL_R		119						// red channel for frame color
#define	FRAME_COL_G		110						// green channel for frame color
#define FRAME_COL_B		111						// blue channel for frame color
//------------------------------------------------------------------------------
#define FILL_COL_R		48						// red channel for fill color
#define	FILL_COL_G		49						// green channel for fill color
#define FILL_COL_B		49						// blue channel for fill colo
//------------------------------------------------------------------------------
#define FORCE_COL_R		255						// red channel for force color
#define FORCE_COL_G		129						// green channel for force color
#define FORCE_COL_B		0						// blue channel for force color
//------------------------------------------------------------------------------
#define SELECTED_COL_R	255						// red channel for selected quad
#define SELECTED_COL_G	0						// green channel for selected quad
#define SELECTED_COL_B	0						// blue channel for selected quad
//------------------------------------------------------------------------------
// PLOT COLOR CONSTANTS
//------------------------------------------------------------------------------
#define PLOT_COL1_R		17						// red channel for plot signal # 1
#define PLOT_COL1_G		150						// green channel for plot signal # 1
#define PLOT_COL1_B		240						// blue channel for plot signal # 1
//------------------------------------------------------------------------------
#define PLOT_COL2_R		251						// red channel for plot signal # 2
#define PLOT_COL2_G		188						// green channel for plot signal # 2
#define PLOT_COL2_B		5						// blue channel for plot signal # 2
//------------------------------------------------------------------------------
#define PLOT_T_AXIS_R  	20						// red channel for plot time axis
#define PLOT_T_AXIS_G	20						// green channel for plot time axis
#define PLOT_T_AXIS_B	20						// blue channel for plot time axis
//------------------------------------------------------------------------------
#define PLOT_TEXT_R  	90						// red channel for plot text
#define PLOT_TEXT_G		90						// green channel for plot text
#define PLOT_TEXT_B		90						// blue channel for plot text
//------------------------------------------------------------------------------
#define PLOT_TICK_R  	90						// red channel for plot text
#define PLOT_TICK_G		90						// green channel for plot text
#define PLOT_TICK_B		90						// blue channel for plot text


//------------------------------------------------------------------------------
//	GLOBAL VARIABLE DEFINITIONS
//------------------------------------------------------------------------------
BITMAP		*fly_bitmap, *plot_x_bitmap, *plot_y_bitmap, *plot_theta_bitmap,\
		    *log_bitmap, *guidance_bitmap;
plot_data	plot_x_est, plot_x_track, plot_y_est, plot_y_track, plot_theta_est,\
		    plot_theta_track;
FONT				*font_16, *font_12, *font_11, *font_10, *font_10_mono;
selected_quadrotor	selected_quad;

//------------------------------------------------------------------------------
//	FUNCTIONS that modifies the variable 'selected_quadrotor'
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function init_selected_quad
//	initializes the variable selected_quadrotor
//------------------------------------------------------------------------------
void	init_selected_quad()
{
		selected_quad.index = 0;
		pthread_mutex_init(&(selected_quad.mutex), NULL);
}

//------------------------------------------------------------------------------
//	Function get_selected_quad
//	returns the index of selected quadrotor
//------------------------------------------------------------------------------
static
int		get_selected_quad()
{
int		index;

		pthread_mutex_lock(&(selected_quad.mutex));
		index = selected_quad.index;
		pthread_mutex_unlock(&(selected_quad.mutex));

		return index;
}

//------------------------------------------------------------------------------
//	Function previous_selected_quad
//	decreases the index of the selected quadrotor
//------------------------------------------------------------------------------
static
void	previous_selected_quad()
{
		pthread_mutex_lock(&(selected_quad.mutex));
		selected_quad.index = modulo(selected_quad.index - 1, MAX_QUADROTORS);
		pthread_mutex_unlock(&(selected_quad.mutex));
}

//------------------------------------------------------------------------------
//	Function next_selected_quad
//	increases the index of the selected quadrotor
//------------------------------------------------------------------------------
static
void	next_selected_quad()
{
		pthread_mutex_lock(&(selected_quad.mutex));
		selected_quad.index = modulo(selected_quad.index + 1, MAX_QUADROTORS);
		pthread_mutex_unlock(&(selected_quad.mutex));
}

//------------------------------------------------------------------------------
//	FONT FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function load_fonts
//	loads all the fonts
//------------------------------------------------------------------------------
static
void	load_fonts()
{
int		i;
char	*font_names[4] = {"opensans_16.pcx", "opensans_12.pcx",\
						  "opensans_11.pcx", "opensans_10.pcx"};
FONT	**fonts[4] = {&font_16, &font_12, &font_11, &font_10};

		 for(i = 0; i < 4; i++)
			*fonts[i] = load_font(font_names[i], NULL, NULL);
}

//------------------------------------------------------------------------------
//	BITMAP FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function bitmap_init
//	initializes all the bitmaps
//------------------------------------------------------------------------------
static
void	bitmap_init()
{
		fly_bitmap = create_bitmap(FLY_W, FLY_H);
		plot_x_bitmap = create_bitmap(PLOT_W, PLOT_H);
		plot_y_bitmap = create_bitmap(PLOT_W, PLOT_H);
		plot_theta_bitmap = create_bitmap(PLOT_W, PLOT_H);
		log_bitmap = create_bitmap(LOG_W, LOG_H);
		guidance_bitmap = create_bitmap(GUIDANCE_W, GUIDANCE_H);
}

//------------------------------------------------------------------------------
//	PLOT DATA FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function plot_init
//	initializes the plot_data 'plot'
//------------------------------------------------------------------------------
static
void	plot_init(plot_data* plot)
{
		plot->index = BUFF_SIZE;
		pthread_mutex_init(&(plot->index_mutex), NULL);
}

//------------------------------------------------------------------------------
//	Function plot_init_all
//	initializes all the plot_data instances
//------------------------------------------------------------------------------
static
void	plot_init_all()
{
			plot_init(&plot_x_est);
			plot_init(&plot_y_est);
			plot_init(&plot_theta_est);
			plot_init(&plot_x_track);
			plot_init(&plot_y_track);
			plot_init(&plot_theta_track);
}

//------------------------------------------------------------------------------
//	Function plot_reset_all
//	resets the plot_data 'plot'
//------------------------------------------------------------------------------
static
void	plot_reset(plot_data* plot)
{
		pthread_mutex_lock(&(plot->index_mutex));
		plot->index = BUFF_SIZE;
		pthread_mutex_unlock(&(plot->index_mutex));
}

//------------------------------------------------------------------------------
//	Function plot_reset_all
//	resets all the plot_data instances
//------------------------------------------------------------------------------
static
void	plot_reset_all()
{
		plot_reset(&plot_x_est);
		plot_reset(&plot_y_est);
		plot_reset(&plot_theta_est);
		plot_reset(&plot_x_track);
		plot_reset(&plot_y_track);
		plot_reset(&plot_theta_track);
}

//------------------------------------------------------------------------------
//	Function plot_add_sample
//  adds a sample to the buffer of the plot_data 'plot'
//------------------------------------------------------------------------------
static
void	plot_add_sample(plot_data* plot, float sample)
{
int 	i;

		pthread_mutex_lock(&(plot->index_mutex));

		// Update the index pointing to the oldest sample
		if(plot->index > 0)
			(plot->index)--;

		// Shift the buffer to the left
		for(i = plot->index; i < BUFF_SIZE - 1; i++)
			plot->buffer[i] = plot->buffer[i + 1];
		// Add the new sample to the buffer
		plot->buffer[BUFF_SIZE - 1] = sample;

		pthread_mutex_unlock(&(plot->index_mutex));
}

//------------------------------------------------------------------------------
//	DRAWING FUNCTION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function draw_quadrotor
//  draws a quadrotor given the coordinates 'x', 'y', 'theta'
//	and the forces 'fl', 'fr'
//------------------------------------------------------------------------------
static
void	draw_quadrotor(BITMAP* bitmap, float x, float y, float theta,\
					   float fl, float fr, int selected)
{
int		fill_color, frame_color, force_color, selected_color;
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
		selected_color = makecol(SELECTED_COL_R, SELECTED_COL_G, SELECTED_COL_B);

		// Draw the quadcopter

		// Triangle fill
		triangle(bitmap, x_left, y_left, x_right, y_right, x_down, y_down, fill_color);

		// Triangle edges
		line(bitmap, x_left, y_left, x_right, y_right, frame_color);
		line(bitmap, x_down, y_down, x_right, y_right, frame_color);
		line(bitmap, x_left, y_left, x_down, y_down, frame_color);

		// Circle
		if (selected)
			circlefill(bitmap, x_centre, y_centre, FLY_SCALING * QUAD_HEIGHT, selected_color);
		circle(bitmap, x_centre, y_centre, FLY_SCALING * QUAD_HEIGHT, frame_color);

		// Force arrows
		line(bitmap, x_left, y_left, x_fl, y_fl, force_color);
		line(bitmap, x_right, y_right, x_fr, y_fr, force_color);
}

//------------------------------------------------------------------------------
//	Function draw_quads
//	draws all quadrotors
//------------------------------------------------------------------------------
static
void	draw_quads()
{
float	x, y, theta, force_left, force_right;
int		i, sky_color, legend_color, selected_quad;

		sky_color = makecol(SKY_COL_R, SKY_COL_G, SKY_COL_B);
 		legend_color = makecol(TEXT_COL_R, TEXT_COL_G, TEXT_COL_B);

		clear_to_color(fly_bitmap, sky_color);

		selected_quad = get_selected_quad();

		// Print the number of the selected quadrotor
		textout_right_ex(fly_bitmap, font_10,"quad # ", FLY_W, FLY_H - MARGIN,
							legend_color, -1);
		textprintf_right_ex(fly_bitmap, font_10, FLY_W, FLY_H - MARGIN,
							legend_color, -1,
							"%d", selected_quad + 1);


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
			draw_quadrotor(fly_bitmap, x, y, theta, force_left, force_right,\
						   get_selected_quad() == i);
		}

		// Transfer image from bitmap to screen
		blit(fly_bitmap, screen, 0, 0, FLY_X, FLY_Y, FLY_W, FLY_H);
}

//------------------------------------------------------------------------------
//	Function draw_layout
//	draws the static layout of the gui
//------------------------------------------------------------------------------
static
void	draw_layout()
{
int		color;

		color = makecol(LAYOUT_COL_R, LAYOUT_COL_G, LAYOUT_COL_B);

		// Draw separators
		line(screen, FLY_X, FLY_Y + FLY_H + MARGIN,\
					 FLY_X + FLY_W, FLY_Y + FLY_H + MARGIN, color);
		line(screen, FLY_X + FLY_W + MARGIN, MARGIN,
					 FLY_X + FLY_W + MARGIN, FLY_Y + FLY_H, color);
		line(screen, FLY_X + FLY_W + 2 * MARGIN, FLY_Y + FLY_H + MARGIN,\
					 FLY_X + FLY_W + 2 * MARGIN + PLOT_TICK_WIDTH +  2 * PLOT_MARGIN + PLOT_W,\
					 FLY_Y + FLY_H + MARGIN, color);
		line(screen, FLY_X + FLY_W + MARGIN, FLY_Y + FLY_H + 2 * MARGIN,
		 			 FLY_X + FLY_W + MARGIN, WINDOW_H - MARGIN, color);

		// Draw figure margins
		rect(screen, PLOT_X_X - PLOT_MARGIN, PLOT_X_Y - PLOT_MARGIN,\
					 PLOT_X_X + PLOT_W + PLOT_MARGIN, PLOT_X_Y + PLOT_H + PLOT_MARGIN,\
					 color);
		rect(screen, PLOT_Y_X - PLOT_MARGIN, PLOT_Y_Y - PLOT_MARGIN,\
					 PLOT_Y_X + PLOT_W + PLOT_MARGIN, PLOT_Y_Y + PLOT_H + PLOT_MARGIN,\
					 color);
		rect(screen, PLOT_TH_X - PLOT_MARGIN, PLOT_TH_Y - PLOT_MARGIN,\
					 PLOT_TH_X + PLOT_W + PLOT_MARGIN, PLOT_TH_Y + PLOT_H + PLOT_MARGIN,\
					 color);
}

//------------------------------------------------------------------------------
//	Function draw_figure_signal
//  plots the signal, stored in the plot_data 'plot', into the BITMAP 'bitmap'
//------------------------------------------------------------------------------
static
void	draw_figure_signal(BITMAP* bitmap, plot_data* plot, float max_value,\
						   int color_r, int color_g, int color_b)
{
int		i, r, g ,b, color;
float	h, s, v;
float	actual_value, next_value;

		// Evaluate main color of the plot
		rgb_to_hsv(color_r, color_g, color_b, &h, &s, &v);

		// Plot the signal
		for(i = plot->index; i < BUFF_SIZE - 1; i++) {

			// Evaluate color with scaled value (hsv)
			hsv_to_rgb(h, s, v * (float)i / BUFF_SIZE, &r, &g, &b);
			color = makecol(r, g, b);

			// Draw the horizontal line
			actual_value = (float)PLOT_H / 2 * (1 - plot->buffer[i] / max_value);
			line(bitmap, i * PLOT_W_SCALE, actual_value, (i + 1) * PLOT_W_SCALE,\
				 actual_value, color);

			// Draw the vertical line
			next_value = (float)PLOT_H / 2 * (1 - plot->buffer[i + 1] / max_value);
			line(bitmap, (i + 1) * PLOT_W_SCALE, actual_value, (i + 1) * PLOT_W_SCALE,\
				 next_value, color);
		}
}

//------------------------------------------------------------------------------
//	Function draw_figure_axes
//	draws the axes of the figure
//------------------------------------------------------------------------------
static
void	draw_figure_axes(BITMAP* bitmap, float scale, float tick)
{
int		i;
int		axis_color;
float	height, width;

		axis_color = makecol(PLOT_T_AXIS_R, PLOT_T_AXIS_G, PLOT_T_AXIS_B);

		// Draw horizontal lines
		for (i = 0; (height = PLOT_H / 2 - i * scale * tick) >= 0; i++)
			line(bitmap, 0, height, PLOT_W, height, axis_color);
		for (i = 1; (height = PLOT_H / 2 + i * scale * tick) <= PLOT_H; i++)
			line(bitmap, 0, height, PLOT_W, height, axis_color);

		// Draw vertical lines
		for (i = 0; (width = i * PLOT_TIME_SCALE * PLOT_TIME_TICK) <= PLOT_W; i++)
			line(bitmap, width, 0, width, PLOT_H, axis_color);
}


//------------------------------------------------------------------------------
//	Function draw_figure_ticks
//	draws the ticks on the figure
//------------------------------------------------------------------------------
static
void	draw_figure_ticks(BITMAP* bitmap, float scale, float tick)
{
int		i;
int		tick_color;
float	height;

		tick_color = makecol(PLOT_TICK_R, PLOT_TICK_G, PLOT_TICK_B);

		// Draw ticks
		for (i = 0; (height = PLOT_H / 2 - i * scale * tick) >= 0; i++)
			line(bitmap, 0, height, 5, height, tick_color);
		for (i = 1; (height = PLOT_H / 2 + i * scale * tick) <= PLOT_H; i++)
			line(bitmap, 0, height, 5, height, tick_color);
}

//------------------------------------------------------------------------------
//	Function draw_figure_title
//  draws the title of the figure
//------------------------------------------------------------------------------
static
void	draw_figure_title(BITMAP* bitmap, char* text)
{
int		color, text_h;

		text_h = text_height(font_10);
		color = makecol(PLOT_TEXT_R, PLOT_TEXT_G, PLOT_TEXT_B);

		textout_right_ex(bitmap, font_10, text, PLOT_W - 2, PLOT_H - text_h,\
				   		 color, -1);
}

//------------------------------------------------------------------------------
//	Function draw_figure
//  draws a figure containing two signals
//------------------------------------------------------------------------------
static
void	draw_figure(BITMAP* bitmap, plot_data* signal_1, plot_data* signal_2,\
					float max_value, int upper_left_x, int upper_left_y,\
					float scale, float tick, char* title)
{
		clear_to_color(bitmap, 0);

		draw_figure_axes(bitmap, scale, tick);

		draw_figure_signal(bitmap, signal_1, max_value, PLOT_COL1_R, PLOT_COL1_G, PLOT_COL1_B);
		draw_figure_signal(bitmap, signal_2, max_value, PLOT_COL2_R, PLOT_COL2_G, PLOT_COL2_B);

		draw_figure_ticks(bitmap, scale, tick);
	   	draw_figure_title(bitmap, title);

		blit(bitmap, screen, 0, 0, upper_left_x, upper_left_y, PLOT_W, PLOT_H);
}

//------------------------------------------------------------------------------
//	Function draw_plot_area
//  draws the plot area containing three figures (estimation and tracking error
//	of x, y and theta for the i-th quadcopter)
//------------------------------------------------------------------------------
static
void	draw_plot_area()
{
float	x, x_est, x_traj;
float	y, y_est, y_traj;
float	theta, theta_est, theta_traj;
int		i;

		i = get_selected_quad();

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

		plot_add_sample(&plot_x_est, x - x_est);
		plot_add_sample(&plot_x_track, x - x_traj);

		plot_add_sample(&plot_y_est, y - y_est);
		plot_add_sample(&plot_y_track, y - y_traj);

		plot_add_sample(&plot_theta_est, theta - theta_est);
		plot_add_sample(&plot_theta_track, theta - theta_traj);

		draw_figure(plot_x_bitmap, &plot_x_est, &plot_x_track, PLOT_X_MAX,\
					PLOT_X_X, PLOT_X_Y, PLOT_X_SCALE, PLOT_X_TICK, \
					"x (m)");
		draw_figure(plot_y_bitmap, &plot_y_est, &plot_y_track, PLOT_Y_MAX,\
					PLOT_Y_X, PLOT_Y_Y, PLOT_Y_SCALE, PLOT_Y_TICK,\
					"y (m)");
		draw_figure(plot_theta_bitmap, &plot_theta_est, &plot_theta_track, PLOT_TH_MAX,\
					PLOT_TH_X, PLOT_TH_Y, PLOT_TH_SCALE, PLOT_TH_TICK,\
					"theta (rad)");
}

//------------------------------------------------------------------------------
//	PRINT FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function print_titles
//	prints titles and subtitles of the gui
//------------------------------------------------------------------------------
static
void	print_titles()
{
int		title_color;

		title_color = makecol(TEXT_COL_R, TEXT_COL_G, TEXT_COL_B);
		textout_ex(screen, font_16, "2D Quad Simulator",\
				   10, 5, title_color, -1);
		textout_centre_ex(screen, font_12, "errors",\
						  PLOT_X_X + PLOT_W / 2, PLOT_X_Y - 25, title_color, -1);
		textout_centre_ex(screen, font_12, "view",\
						  FLY_X + FLY_W / 2, FLY_Y - 25, title_color, -1);
		textout_centre_ex(screen, font_12, "realtime peformance",\
						  FLY_X + FLY_W / 2, FLY_Y + FLY_H + MARGIN + 10, title_color, -1);
		textout_centre_ex(screen, font_12, "guidance",\
						  PLOT_X_X + PLOT_W / 2, FLY_Y + FLY_H + MARGIN + 10, title_color, -1);
}

//------------------------------------------------------------------------------
//	Function print_plot_legend
//	prints the lengend of the plot area
//------------------------------------------------------------------------------
static
void	print_plot_legend()
{
int		legend_color, tracking_error_color, estimation_error_color;
int		second_word_length;

		// Set colors
 		legend_color = makecol(TEXT_COL_R, TEXT_COL_G, TEXT_COL_B);
	   	estimation_error_color = makecol(PLOT_COL1_R, PLOT_COL1_G, PLOT_COL1_B);
		tracking_error_color = makecol(PLOT_COL2_R, PLOT_COL2_G, PLOT_COL2_B);

		// Print texts
		textout_ex(screen, font_10, "legend:",\
				   PLOT_X_X - PLOT_MARGIN,\
				   PLOT_TH_Y + PLOT_H + PLOT_MARGIN + 6, legend_color, -1);

		textout_right_ex(screen, font_10, "estimation err",\
				   		 PLOT_X_X + PLOT_W + PLOT_MARGIN,\
				   		 PLOT_TH_Y + PLOT_H + PLOT_MARGIN + 6, estimation_error_color, -1);

	 	second_word_length = text_length(font_10, "estimation err ");
		textout_right_ex(screen, font_10, "tracking err",\
						 PLOT_X_X + PLOT_W + PLOT_MARGIN - second_word_length, \
						 PLOT_TH_Y + PLOT_H + PLOT_MARGIN + 6, tracking_error_color, -1);

		textout_ex(screen, font_10, "legend:",\
				   PLOT_X_X - PLOT_MARGIN,\
				   PLOT_TH_Y + PLOT_H + PLOT_MARGIN + 6, legend_color, -1);
}

//------------------------------------------------------------------------------
//	Function print_plot_ticks
//	prints the text of the ticks for the figure whose 0-axis is at height
//	'height_base' given the axis scale and tick length
//------------------------------------------------------------------------------
static
void	print_plot_ticks(float height_base, float scale, float tick)
{
int		i, text_color;
float	width, height, h_upper, h_lower, text_h_offset;

		text_color = makecol(PLOT_TEXT_R, PLOT_TEXT_G, PLOT_TEXT_B);
		width = PLOT_X_X - 5;
		h_lower = height_base - PLOT_H / 2;
		h_upper = height_base + PLOT_H / 2;
		text_h_offset = -14;

		// Draw ticks
		for (i = 0; (height = height_base - i * scale * tick) >= h_lower; i++)
			textprintf_right_ex(screen, font_10, width - 2, height + text_h_offset,
								text_color, -1, "%.2f", i * tick);

		for (i = 1; (height = height_base + i * scale * tick) <= h_upper; i++)
			textprintf_right_ex(screen, font_10, width - 2, height + text_h_offset,
								text_color, -1, "-%.2f", i * tick);
}

//------------------------------------------------------------------------------
//	Function print_plot_ticks_all
//	prints the text of the ticks for each plot in the plot area
//------------------------------------------------------------------------------
static
void	print_plot_ticks_all()
{
float	height_base, height_step;

		height_base = PLOT_X_Y + PLOT_MARGIN + PLOT_H / 2;
		height_step = PLOT_H + 2 * PLOT_MARGIN + PLOT_SPACING;

		print_plot_ticks(height_base, PLOT_X_SCALE, PLOT_X_TICK);
		print_plot_ticks(height_base + height_step, PLOT_Y_SCALE, PLOT_Y_TICK);
		print_plot_ticks(height_base + 2 * height_step, PLOT_TH_SCALE, PLOT_TH_TICK);
}

//------------------------------------------------------------------------------
//	Function print_performace_label
//	prints a label containing realtime performance indicators
//	the label is drawn at ('x', 'y')
//------------------------------------------------------------------------------
static
void   	print_performace_label(BITMAP* bitmap, int x, int y, task_par* tp)
{
int 	height, text_color, deadline_miss;
float	wcet;

		// Set color
		text_color = makecol(TEXT_COL_R, TEXT_COL_G, TEXT_COL_B);

		// Evaluate text_width and text_height
		height = text_height(font_11);

		pthread_mutex_lock(&(tp->mutex));
		deadline_miss = tp->dmiss;
		wcet = time_to_ms(&(tp->wcet));
		pthread_mutex_unlock(&(tp->mutex));

		textprintf_ex(bitmap, font_11, x, y, text_color, -1, "%s", tp->task_name);
		textprintf_ex(bitmap, font_10, x, y + height, text_color, -1,\
					  "period: %d ms", tp->period);
		textprintf_ex(bitmap, font_10, x, y + 2 * height, text_color, -1,\
					  "wcet: %.4f", wcet);
		textprintf_ex(bitmap, font_10, x, y + 2 * height, text_color, -1,\
					  "%16s", "ms");
		textprintf_ex(bitmap, font_10, x, y + 3 * height, text_color, -1,\
					  "deadline miss: %d", deadline_miss);
}

//------------------------------------------------------------------------------
//	Function print_performace_info
//	prints the performance info for all the tasks
//------------------------------------------------------------------------------
static
void	print_performace_info()
{
int		i;

		i = get_selected_quad();

		clear_to_color(log_bitmap, 0);

		print_performace_label(log_bitmap, 0, 0, &(gui_tp[0]));
		print_performace_label(log_bitmap, LOG_SPACING, 0,  &(regulator_tp[i]));
		print_performace_label(log_bitmap, 2 * LOG_SPACING, 0, &(guidance_tp[i]));

		blit(log_bitmap, screen, 0, 0, LOG_X, LOG_Y, LOG_W, LOG_H);
}

//------------------------------------------------------------------------------
//	Function print_guidance_info
//	prints the guidance info for the selected quadcopter
//------------------------------------------------------------------------------
static
void	print_guidance_info()
{
int		i, text_color, height;
float	x, y, xf, yf, tof;

		i = get_selected_quad();

		clear_to_color(guidance_bitmap, 0);

		// Set color
		text_color = makecol(TEXT_COL_R, TEXT_COL_G, TEXT_COL_B);

		// Evaluate text_width and text_height
		height = text_height(font_11);

		pthread_mutex_lock(&dynamics_mutex[i]);
		x = states[i].x;
		y = states[i].y;
		pthread_mutex_unlock(&dynamics_mutex[i]);

		pthread_mutex_lock(&guidance_mutex[i]);
		xf = traj_states[i].xf;
		yf = traj_states[i].yf;
		tof = traj_states[i].final_time;
		pthread_mutex_unlock(&guidance_mutex[i]);

		if (get_guidance_state(i))
			textprintf_ex(guidance_bitmap, font_10, 0, 0, text_color, -1, "Active: Yes");
		else
			textprintf_ex(guidance_bitmap, font_10, 0, 0, text_color, -1, "Active: No");
		textprintf_ex(guidance_bitmap, font_10, 0, height, text_color, -1,\
					  "Next target: [%.2f", xf);
		textprintf_ex(guidance_bitmap, font_10, 0, height, text_color, -1,\
					  "%21s%.2f", ", ", yf);
		textprintf_ex(guidance_bitmap, font_10, 0, height, text_color, -1,\
					  "%28s", "] m");
	 	textprintf_ex(guidance_bitmap, font_10, 0, 2 * height, text_color, -1,\
  					  "State : [%.2f", x);
	 	textprintf_ex(guidance_bitmap, font_10, 0, 2 * height, text_color, -1,\
  					  "%15s%.2f", ", ", y);
		textprintf_ex(guidance_bitmap, font_10, 0, 2 * height, text_color, -1,\
					  "%22s", "] m");
		textprintf_ex(guidance_bitmap, font_10, 0, 3 * height, text_color, -1,\
  					  "Time of flight: %.2f", tof);
		textprintf_ex(guidance_bitmap, font_10, 0, 3 * height, text_color, -1,\
					  "%22s", "s");

		blit(guidance_bitmap, screen, 0, 0, GUIDANCE_X, GUIDANCE_Y, GUIDANCE_W, GUIDANCE_H);
}

//------------------------------------------------------------------------------
//	TASK CODE
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	GUI TASK
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function gui_task
//	performs initializations required for the GUI
//------------------------------------------------------------------------------
static
void	gui_init()
{
int		bg_color;

			bg_color = makecol(BG_COL_R, BG_COL_G, BG_COL_B);

			load_fonts();
			bitmap_init();
			plot_init_all();
			clear_to_color(screen, bg_color);
			draw_layout();
			print_titles();
			print_plot_legend();
			print_plot_ticks_all();
}

//------------------------------------------------------------------------------
//	Function gui_task
//------------------------------------------------------------------------------
void*		gui_task(void* arg)
{
task_par*	tp;

			tp = (struct task_par*)arg;
			init_timespecs(tp);

			gui_init();

			while(1) {

				set_start_time(tp);

				draw_quads();
				draw_plot_area();
				print_performace_info();
				print_guidance_info();

				// Handle thread parameters
				set_finish_time(tp);
				update_wcet(tp);
				deadline_miss(tp);
				wait_for_period(tp);
				update_activation_time(tp);
				update_abs_deadline(tp);
			}
}

//------------------------------------------------------------------------------
//	USER TASK
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function exec_user_command
//	reads the key pressed by the user and perform the corresponding action
//------------------------------------------------------------------------------
static
void	exec_user_command()
{
		readkey();
		if(key[KEY_P]) {
			previous_selected_quad();
			plot_reset_all();
		}
		else if(key[KEY_N]) {
			next_selected_quad();
			plot_reset_all();
		}
		else if(key[KEY_G]) {
			switch_guidance(get_selected_quad());
		}
		else if(key[KEY_Q])
			exit(EXIT_SUCCESS);
}

//------------------------------------------------------------------------------
//	Function user_task
//------------------------------------------------------------------------------
void*		user_task(void* arg)
{
task_par*	tp;

			tp = (struct task_par*)arg;

			while(1) {
				exec_user_command();

				aperiodic_wait(tp);
			}
}

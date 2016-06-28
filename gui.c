//------------------------------------------------------------------------------
//	GUI.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <allegro.h>
#include <stdio.h>
#include "simulator.h"
#include "utils.h"

//------------------------------------------------------------------------------
//	FLYING AREA CONSTANTS
//------------------------------------------------------------------------------
#define GRAPHIC_W	1024  					// horizontal bound in pixel
#define GRAPHIC_H	576						// vertical bound in pixel
#define	SCALING		GRAPHIC_W / WORLD_W		// scale factor
//------------------------------------------------------------------------------
//	QUADROTOR GRAPHIC CONSTANTS
//------------------------------------------------------------------------------
#define	QUAD_HEIGHT		0.07				// height of the quadcopter in m
#define	F_THS			M * G / 2			// force threshold in N
#define	MAX_F_ARR		0.26				// max length of arrow representing force in m
#define	FORCE_SCALE		-MAX_F_ARR /(F_THS) // scale factor for force representation
//------------------------------------------------------------------------------
//	COLOR CONSTANTS
//------------------------------------------------------------------------------
#define FILL_COL_R		48					// red channel for fill color
#define	FILL_COL_G		49					// green channel for fill color
#define FILL_COL_B		49					// blue channel for fill color
#define FRAME_COL_R		119					// red channel for frame color
#define	FRAME_COL_G		110					// green channel for frame color
#define FRAME_COL_B		111					// blue channel for frame color
#define GROUND_COL_R	66					// red channel for frame color
#define	GROUND_COL_G	76					// green channel for frame color
#define GROUND_COL_B	68					// blue channel for frame color
#define FORCE_COL_R		255					// red channel for frame color
#define FORCE_COL_G		0					// red channel for frame color
#define FORCE_COL_B		0					// red channel for frame color
//------------------------------------------------------------------------------
//	Function draw_quadrotor
//------------------------------------------------------------------------------
void	draw_quadrotor(float x, float y, float theta, float fl, float fr)
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
		scale(scale_, SCALING);
		mat_mul(3, 3, scale_, 3, 3, transl, transformation);
		mat_mul(3, 3, transformation, 3, 3, rot, transformation);
		mat_mul(3, 3, transformation, 3, 6, coords, coords);

		x_left = coords[0][0];
		x_right = coords[0][1];
		x_down = coords[0][2];
		x_centre = coords[0][3];
		x_fl = coords[0][4];
		x_fr = coords[0][5];
		y_left = GRAPHIC_H - coords[1][0];
		y_right = GRAPHIC_H - coords[1][1];
		y_down = GRAPHIC_H - coords[1][2];
		y_centre = GRAPHIC_H - coords[1][3];
		y_fl = GRAPHIC_H - coords[1][4];
		y_fr = GRAPHIC_H - coords[1][5];

		// Set colors
		fill_color = makecol(FILL_COL_R, FILL_COL_G, FILL_COL_B);
		frame_color = makecol(FRAME_COL_R, FRAME_COL_G, FRAME_COL_B);
		force_color = makecol(FORCE_COL_R, FORCE_COL_G, FORCE_COL_B);

		// Draw the quadcopter
		// Body
		triangle(screen, x_left, y_left, x_right, y_right, x_down, y_down, fill_color);
		circle(screen, x_centre, y_centre, SCALING * QUAD_HEIGHT, frame_color);
		// Edges
		line(screen, x_left, y_left, x_right, y_right, frame_color);
		line(screen, x_down, y_down, x_right, y_right, frame_color);
		line(screen, x_left, y_left, x_down, y_down, frame_color);
		// Forces
		line(screen, x_left, y_left, x_fl, y_fl, force_color);
		line(screen, x_right, y_right, x_fr, y_fr, force_color);

}
//------------------------------------------------------------------------------
//	Function gui_task
//------------------------------------------------------------------------------
void*	gui_task(void* arg)
{
struct task_par* tp;
float	x, y, theta, force_left, force_right;
int		i;
		tp = (struct task_par*)arg;

		init_timespecs(tp);

		// Allegro init
		allegro_init();
		set_color_depth(8);
		set_gfx_mode(GFX_AUTODETECT_WINDOWED, GRAPHIC_W, GRAPHIC_H, 0, 0);

		while(1) {

			clear_to_color(screen, 0);
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
				draw_quadrotor(x, y, theta, force_left, force_right);
			}

			// Handle thread parameters
			if(deadline_miss(tp))
				printf("%d\n", tp->dmiss);
			wait_for_period(tp);
			update_activation_time(tp);
			update_abs_deadline(tp);
		}
}


//------------------------------------------------------------------------------
//	GUI.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <allegro.h>
#include "simulator.h"
#include "utils.h"
#include "unistd.h"

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

//------------------------------------------------------------------------------
//	Function draw_quadrotor
//------------------------------------------------------------------------------
void	draw_quadrotor(float x, float y, float theta)
{
float	rot[3][3], transl[3][3], scale_[3][3], transformation[3][3];
float	coordinates[3][4] = {{L, - L, 0, 0}, {0, 0, - QUAD_HEIGHT, 0}, {1, 1, 1, 1}};
float	x_left, x_right, x_down, x_centre, y_left, y_right, y_down, y_centre;
int		fill_color, frame_color;

		// Transform body fixed coordinates into world coordinates
		// New coordinates are stored in 'coordinates'
		rotate(rot, theta);
		translate(transl, x, y);
		scale(scale_, SCALING);
		mat_mul(3, 3, scale_, 3, 3, transl, transformation);
		mat_mul(3, 3, transformation, 3, 3, rot, transformation);
		mat_mul(3, 3, transformation, 3, 4, coordinates, coordinates);

		x_left = coordinates[0][0];
		x_right = coordinates[0][1];
		x_down = coordinates[0][2];
		x_centre = coordinates[0][3];
		y_left = GRAPHIC_H - coordinates[1][0];
		y_right = GRAPHIC_H - coordinates[1][1];
		y_down = GRAPHIC_H - coordinates[1][2];
		y_centre = GRAPHIC_H - coordinates[1][3];

		fill_color = makecol(FILL_COL_R, FILL_COL_G, FILL_COL_B);
		frame_color = makecol(FRAME_COL_R, FRAME_COL_G, FRAME_COL_B);

		// Draw the quadcopter
		// Body
		triangle(screen, x_left, y_left, x_right, y_right, x_down, y_down, fill_color);
		circle(screen, x_centre, y_centre, SCALING * QUAD_HEIGHT, frame_color);
		// Edges
		line(screen, x_left, y_left, x_right, y_right, frame_color);
		line(screen, x_down, y_down, x_right, y_right, frame_color);
		line(screen, x_left, y_left, x_down, y_down, frame_color);
		// Forces
		// TODO
}

/* int		main() */
/* { */
/* float	x, y, theta; */

/* 		x = 1; */
/* 		y = 3; */
/* 		theta = 0; */

/* 		// Allegro init */
/* 		allegro_init(); */
/* 		set_color_depth(8); */
/* 		set_gfx_mode(GFX_AUTODETECT_WINDOWED, GRAPHIC_W, GRAPHIC_H, 0, 0); */

/* 		while(1) */
/* 		{ */
/* 			clear_to_color(screen, 0); */
/* 			draw_quadrotor(x, y, theta); */
/* 			theta += 0.785 / 30; */
/* 			usleep(33333); */
/* 		} */
/* 		allegro_exit(); */
/* 		return 0; */
/* } */

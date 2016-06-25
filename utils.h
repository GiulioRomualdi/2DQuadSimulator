#ifndef UTILS_H
#define UTILS_H

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	MATRIX HANDLING FUNCTION
//------------------------------------------------------------------------------
void	mat_zero(int n, int m, float matrix[][m]);
void	mat_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void 	mat_sub(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void	mat_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   	    float dst[op1_n][op2_m]);
void 	mat_transpose(int n, int m, float src[][m], float dst[][n]);
void	mat_3_inv(float matrix[3][3], float dst[3][3]);
void	rotate(float matrix[3][3], float angle);
void	translate(float matrix[3][3], float x, float y);
void	scale(float matrix[3][3], float k);

//------------------------------------------------------------------------------
//	RANDOM NUMBERS GENERATION
//------------------------------------------------------------------------------
void	init_random_generator();
float	get_gaussian(float std);

#endif

#ifndef UTILS_H
#define UTILS_H

//------------------------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	MATRIX HANDLING FUNCTION
//------------------------------------------------------------------------------
void	matrix_zero(int n, int m, float matrix[][m]);
void	matrix_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void 	matrix_sub(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void	matrix_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   	    float dst[op1_n][op2_m]);
void 	matrix_transpose(int n, int m, float src[][m], float dst[][n]);
void	matrix_3_inv(float matrix[3][3], float dst[3][3]);

//------------------------------------------------------------------------------
//	RANDOM NUMBERS GENERATION
//------------------------------------------------------------------------------
void	init_random_generator();
float	get_gaussian(float std);

#endif

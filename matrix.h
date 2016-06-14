#ifndef MATRIX_H
#define MATRIX_H

void	matrix_sum(int n, int m, float op1[][m], float op2[][m], float res[][m]);
void	matrix_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   	    float res[op1_n][op2_m]);
float	matrix_3_det(float matrix[3][3]);
void	matrix_3_adj(float matrix[3][3], float res[3][3]);
void	matrix_3_inv(float matrix[3][3], float res[3][3]);

#endif

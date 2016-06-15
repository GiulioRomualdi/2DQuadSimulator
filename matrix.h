#ifndef MATRIX_H
#define MATRIX_H

void	matrix_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m]);
void	matrix_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   	    float dst[op1_n][op2_m]);
void 	matrix_transpose(int n, int m, float src[][m], float dst[][n]);
void	matrix_3_inv(float matrix[3][3], float dst[3][3]);

#endif

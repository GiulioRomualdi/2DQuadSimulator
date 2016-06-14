#include "matrix.h"

//------------------------------------------------------------------------------
//	Function matrix sum
//	evaluates the sum of two matrices op1 and op2
//	and put the result in res
//------------------------------------------------------------------------------
void	matrix_sum(int n, int m, float op1[][m], float op2[][m], float res[][m])
{
int 	i, j;

		for (i = 0; i < n; i++)
			for (j = 0; j < m; j++)
				res[i][j] = op1[i][j] + op2[i][j];
}

//------------------------------------------------------------------------------
//	Function matrix_mul
//	evaluates the product of two matrices op1 and op2
//	and put the result in res
//------------------------------------------------------------------------------
void	matrix_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   		float res[op1_n][op2_m])
{
int		i, j, k;
float 	a_ij;
		for (i = 0; i < op1_n; i++)
			for (j = 0; j < op2_m; j++)
			{
				a_ij = 0;
				for (k = 0; k < op1_m; k++)
					a_ij += op1[i][k] * op2[k][j];
				res[i][j] = a_ij;
			}
}

//------------------------------------------------------------------------------
//	Function matrix_3_det
//	returns the determinant of a 3 by 3 matrix (closed form used)
//------------------------------------------------------------------------------
float	matrix_3_det(float m[3][3])
{
float	result;
		result = m[0][0] * (m[2][2] * m[1][1] - m[2][1] * m[1][2]) -\
				m[1][0] * (m[2][2] * m[0][1] - m[2][1] * m[0][2]) +\
				m[2][0] * (m[1][2] * m[0][1] - m[1][1] * m[0][2]);

		return result;
}

//------------------------------------------------------------------------------
//	Function matrix_3_adj
//	evaluates the adjoint of a 3 by 3 matrix (closed form used)
//	and put the result in res
//------------------------------------------------------------------------------
void	matrix_3_adj(float m[3][3], float res[3][3])
{
		res[0][0] = m[2][2] * m[1][1] - m[2][1] * m[1][2];
		res[0][1] = -(m[2][2] * m[0][1] - m[2][1] * m[0][2]);
		res[0][2] = m[1][2] * m[0][1] - m[1][1] * m[0][2];
		res[1][0] = -(m[2][2] * m[1][0] - m[2][0] * m[1][2]);
		res[1][1] = m[2][2] * m[0][0] - m[2][0] * m[0][2];
		res[1][2] = -(m[1][2] * m[0][0] - m[1][0] * m[0][2]);
		res[2][0] = m[2][1] * m[1][0] - m[2][0] * m[1][1];
		res[2][1] = -(m[2][1] * m[0][0] - m[2][0] * m[0][1]);
		res[2][2] = m[1][1] * m[0][0] - m[1][0] * m[0][1];
}

//------------------------------------------------------------------------------
//	Function matrix_3_inv
//	evaluates the inverse of a 3 by 3 matrix (closed form used)
//	and put the result in res
//------------------------------------------------------------------------------
void	matrix_3_inv(float matrix[3][3], float res[3][3])
{
int		i, j;
float	determinant;

		matrix_3_adj(matrix, res);
		determinant = matrix_3_det(matrix);

		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				res[i][j] /= determinant;
}

//------------------------------------------------------------------------------
//	UTILS.C:	DESCRIPTION
//------------------------------------------------------------------------------

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "utils.h"
#include "simulator.h"

//------------------------------------------------------------------------------
//	CONSTANTS
//------------------------------------------------------------------------------
#define NSUM 1000	//number of RV used in gaussian random generator

//------------------------------------------------------------------------------
//	GLOABAL VARIABLE DEFINITIONS
//------------------------------------------------------------------------------
//	Mutex required for mutual exclusion
pthread_mutex_t guidance_mutex[MAX_QUADROTORS];
pthread_mutex_t dynamics_mutex[MAX_QUADROTORS];
pthread_mutex_t kalman_mutex[MAX_QUADROTORS];
pthread_mutex_t force_mutex[MAX_QUADROTORS];

//------------------------------------------------------------------------------
//	MATRIX HANDLING FUNCTION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function mat_zero
//	set to zero all the elements of the given matrix
//	MOVE ME OUT OF HERE!
//------------------------------------------------------------------------------
void	mat_zero(int n, int m, float matrix[][m])
{
int		i, j;

		for (i = 0; i < n; i++)
			for (j = 0; j < m; j++)
				matrix[i][j] = 0;
}

//------------------------------------------------------------------------------
//	Function mat_copy
//	copies the matrix 'src' into the matrix 'dst'
//------------------------------------------------------------------------------
static
void	mat_copy(int n, int m, float src[][m], float dst[][m])
{
int		i, j;

	for (i = 0; i < n; i++)
		for (j = 0; j < m; j++)
			dst[i][j] = src[i][j];
}

//------------------------------------------------------------------------------
//	Function mat_sum_scalar
//	evaluates the sum of matrices 'op1' + alpha * 'op2'
//	and put the result in 'dst'
//------------------------------------------------------------------------------
static
void	mat_sum_scalar(int n, int m, float op1[][m], float op2[][m],\
						  float dst[][m], float alpha)
{
int 	i, j;

		for (i = 0; i < n; i++)
			for (j = 0; j < m; j++)
				dst[i][j] = op1[i][j] + alpha * op2[i][j];
}

//------------------------------------------------------------------------------
//	Function mat sum
//	evaluates the sum of two matrices 'op1' and 'op2'
//	and put the result in 'dst'
//------------------------------------------------------------------------------
void	mat_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m])
{
		mat_sum_scalar(n, m, op1, op2, dst, 1);
}

//------------------------------------------------------------------------------
//	Function mat sub
//	evaluates the difference of two matrices 'op1' and 'op2'
//	and put the result in 'dst'
//------------------------------------------------------------------------------
void	mat_sub(int n, int m, float op1[][m], float op2[][m], float dst[][m])
{
		mat_sum_scalar(n, m, op1, op2, dst, -1);
}

//------------------------------------------------------------------------------
//	Function mat_mul
//	evaluates the product of two matrices 'op1' and 'op2'
//	and put the result in 'dst'
//------------------------------------------------------------------------------
void	mat_mul(int op1_n, int op1_m, float op1[][op1_m],\
					int op2_n, int op2_m, float op2[][op2_m],\
			   		float dst[op1_n][op2_m])
{
int		i, j, k;
float 	a_ij;
float	op1_copy[op1_n][op1_m], op2_copy[op2_n][op2_m];

		mat_copy(op1_n, op1_m, op1, op1_copy);
		mat_copy(op2_n, op2_m, op2, op2_copy);

		for (i = 0; i < op1_n; i++)
			for (j = 0; j < op2_m; j++) {
				a_ij = 0;
				for (k = 0; k < op1_m; k++)
					a_ij += op1_copy[i][k] * op2_copy[k][j];
				dst[i][j] = a_ij;
			}
}

//------------------------------------------------------------------------------
//	Function mat_transpose
//	evaluates the transpose of the matrix 'src'
//	and put the result in 'dst'
//------------------------------------------------------------------------------
void	mat_transpose(int n, int m, float src[][m], float dst[][n])
{
int 	i, j;
float	src_copy[n][m];

		mat_copy(n, m, src, src_copy);

		for (i = 0; i < n; i++)
			for (j = 0; j < m; j++)
				dst[j][i] = src_copy[i][j];
}

//------------------------------------------------------------------------------
//	Function mat_3_det
//	returns the determinant of the 3 by 3 matrix 'm' (closed form used)
//------------------------------------------------------------------------------
static
float	mat_3_det(float m[3][3])
{
float	result;

		result = m[0][0] * (m[2][2] * m[1][1] - m[2][1] * m[1][2]) -\
				m[1][0] * (m[2][2] * m[0][1] - m[2][1] * m[0][2]) +\
				m[2][0] * (m[1][2] * m[0][1] - m[1][1] * m[0][2]);

		return result;
}

//------------------------------------------------------------------------------
//	Function mat_3_adj
//	evaluates the adjoint of the 3 by 3 matrix 'src' (closed form used)
//	and put the result in 'dst'
//------------------------------------------------------------------------------
static
void	mat_3_adj(float src[3][3], float dst[3][3])
{
float	m[3][3];

		mat_copy(3, 3, src, m);

		dst[0][0] = m[2][2] * m[1][1] - m[2][1] * m[1][2];
		dst[0][1] = -(m[2][2] * m[0][1] - m[2][1] * m[0][2]);
		dst[0][2] = m[1][2] * m[0][1] - m[1][1] * m[0][2];
		dst[1][0] = -(m[2][2] * m[1][0] - m[2][0] * m[1][2]);
		dst[1][1] = m[2][2] * m[0][0] - m[2][0] * m[0][2];
		dst[1][2] = -(m[1][2] * m[0][0] - m[1][0] * m[0][2]);
		dst[2][0] = m[2][1] * m[1][0] - m[2][0] * m[1][1];
		dst[2][1] = -(m[2][1] * m[0][0] - m[2][0] * m[0][1]);
		dst[2][2] = m[1][1] * m[0][0] - m[1][0] * m[0][1];
}

//------------------------------------------------------------------------------
//	Function mat_3_inv
//	evaluates the inverse of the 3 by 3 matrix 'matrix' (closed form used)
//	and put the result in 'dst'
//------------------------------------------------------------------------------
void	mat_3_inv(float matrix[3][3], float dst[3][3])
{
int		i, j;
float	determinant;

		determinant = mat_3_det(matrix);
		mat_3_adj(matrix, dst);

		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				dst[i][j] /= determinant;
}

//------------------------------------------------------------------------------
//	Function rotate
//	builds a rotation 3x3 matrix created from an agle expressed in radians (SE(2))
//	the axis of rotation is the z axis
//------------------------------------------------------------------------------
void	rotate(float matrix[3][3], float angle)
{
float	sine, cosine;

	   	sine = sin(angle);
	   	cosine = cos(angle);

		matrix[0][0] = cosine;
		matrix[0][1] = - sine;
		matrix[0][2] = 0;
		matrix[1][0] = sine;
		matrix[1][1] = cosine;
		matrix[1][2] = 0;
		matrix[2][0] = 0;
		matrix[2][1] = 0;
		matrix[2][2] = 1;
}

//------------------------------------------------------------------------------
//	Function translate
//	builds a tralation 3x3 matrix created from 2 scalars (SE(2))
//------------------------------------------------------------------------------
void	translate(float matrix[3][3], float x, float y)
{
		mat_zero(3, 3, matrix);

		matrix[0][0] = 1;
		matrix[0][2] = x;
		matrix[1][1] = 1;
		matrix[1][2] = y;
		matrix[2][2] = 1;
}

//------------------------------------------------------------------------------
//	Function scale
//	builds a scaling 3x3 matrix created from a scaling factor (SE(2))
//------------------------------------------------------------------------------
void	scale(float matrix[3][3], float k)
{
		mat_zero(3, 3, matrix);

		matrix[0][0] = k;
		matrix[1][1] = k;
		matrix[2][2] = 1;
}

//------------------------------------------------------------------------------
//	RANDOM NUMBERS GENERATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function init_random_generator
//	initializes random number generator using srand()
//------------------------------------------------------------------------------
void	init_random_generator()
{
		srand(time(NULL));
}

//------------------------------------------------------------------------------
//	Function get_uniform
//	returns a sample taken from a uniform distribution U(0, T) given T
//------------------------------------------------------------------------------
float	get_uniform(float T)
{
float	sample;

		// Generate a uniform sample in [0, 1]
		sample = (float)rand() / RAND_MAX;
		// Scale the sample by T
		sample *= T;

		return sample;
}

//------------------------------------------------------------------------------
//	Function get_uniform_generic
//	returns a sample taken from a uniform distribution U(a, b) given a and b
//------------------------------------------------------------------------------
float	get_uniform_generic(float a, float b)
{
float	sample;

		// Generate a uniform sample in [0, 1]
		sample = get_uniform(b - a);
		sample += a;

		return sample;
}

//------------------------------------------------------------------------------
//	Function get_uniform_from_std
//	returns a sample taken from a uniform distribution U(-T/2, T/2)
//	T = std / sqrt(12) where 'std' is the standard deviation
//------------------------------------------------------------------------------
static
float	get_uniform_from_std(float std)
{
float	T, sample;

		// if the standard deviation is 0 the interval length shrinks to 0
		if (std == 0)
			return 0;

		// The length of the interval is std * sqrt(12)
		T = std * sqrt(12);
		// Forces the sample to be in the range [-T/2, T/2]
		// sample = fmod(rand(), T) - T / 2;
		sample = get_uniform(T) - T / 2;

		return sample;
}

//------------------------------------------------------------------------------
//	Function get_gaussian
//	returns a sample taken from a zero mean gauassian distribution N(0, std^2)
//	(uses the Central Limit Theorem)
//------------------------------------------------------------------------------
float	get_gaussian(float std)
{
int		i;
float	sample;

		sample = 0;
		// Evaluates the sample using the Central Limit Theorem
		for (i = 0; i < NSUM; i++)
			sample += get_uniform_from_std(std);
		sample /= sqrt(NSUM);

		return sample;
}

//------------------------------------------------------------------------------
//	TIME HANDLING
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function time_copy
//	copies a source timespec 't_source' in a destination timespec pointed by 
//	't_dest'
//------------------------------------------------------------------------------
void	time_copy(struct timespec t_source, struct timespec* t_dest)
{
		t_dest->tv_sec = t_source.tv_sec;
		t_dest->tv_nsec = t_source.tv_nsec;
}

//------------------------------------------------------------------------------
//	Function time_add_delta
//	adds a value in milliseconds 'delta' to the time represented
//	by the timespec pointed by 'time'
//------------------------------------------------------------------------------
void	time_add_delta(struct timespec* time, int delta)
{
		time->tv_sec += delta / 1000;
		time->tv_nsec += (delta % 1000) * 1000000;

		if (time->tv_nsec > 1000000000) {
			time->tv_nsec -= 1000000000;
			time->tv_sec += 1;
		}
}

//------------------------------------------------------------------------------
//	Function time_cmp
//	compares two time variables represented by two timespec 't1' and 't2'
//	and returns 0 if they are equal, 1 if 't1' > 't2', -1 if 't1' < 't2'
//------------------------------------------------------------------------------
int		time_cmp(struct timespec t1, struct timespec t2)
{
		if(t1.tv_sec > t2.tv_sec)
			return 1;
		if(t1.tv_sec < t2.tv_sec)
			return -1;
		if(t1.tv_nsec > t2.tv_nsec)
			return 1;
		if(t1.tv_nsec < t2.tv_nsec)
			return -1;
		return 0;
}

//-----------------------------------------------------------------------------
//	THREAD MANAGEMENT
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//	Function mutex_init
//	initializes all the mutex required in the application
//-----------------------------------------------------------------------------
void	mutex_init()
{
int		i;

		 for(i = 0; i < MAX_QUADROTORS; i++) {
			pthread_mutex_init(&guidance_mutex[i], NULL);
			pthread_mutex_init(&dynamics_mutex[i], NULL);
			pthread_mutex_init(&kalman_mutex[i], NULL);
			pthread_mutex_init(&force_mutex[i], NULL);
		 }
}

//-----------------------------------------------------------------------------
//	Function init_timespecs
//	read the current time and computes the next activation time
//	and the absolute deadline of the task
//	the timespecs 'activation_time' and 'abs_deadline' are set accordingly
//-----------------------------------------------------------------------------
void	init_timespecs(struct task_par* tp)
{
struct timespec time;

		clock_gettime(CLOCK_MONOTONIC, &time);
		time_copy(time, &(tp->activation_time));
		time_copy(time, &(tp->abs_deadline));
		time_add_delta(&(tp->activation_time), tp->period);
		time_add_delta(&(tp->abs_deadline), tp->deadline);
}

//-----------------------------------------------------------------------------
//	Function update_activation_time
//	updates the activation time
//-----------------------------------------------------------------------------
void	update_activation_time(struct task_par* tp)
{
		time_add_delta(&(tp->activation_time), tp->period);
}

//-----------------------------------------------------------------------------
//	Function update_abs_deadline
//	updates the absolute deadline
//-----------------------------------------------------------------------------
void	update_abs_deadline(struct task_par* tp)
{
		time_add_delta(&(tp->abs_deadline), tp->period);
}

//-----------------------------------------------------------------------------
//	Function wait_for_period
//	suspends the thread until the next activation
//-----------------------------------------------------------------------------
void	wait_for_period(struct task_par* tp)
{
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->activation_time),\
						NULL);
}

//-----------------------------------------------------------------------------
//	Function deadline_miss
//	compare the current execution time with the expected absolute deadline
//	returns 1 in case of deadline miss and increments 'dmiss'
//	otherwise it returns 0
//-----------------------------------------------------------------------------
int		deadline_miss(struct task_par* tp)
{
struct timespec current_time;

		clock_gettime(CLOCK_MONOTONIC, &current_time);

		if(time_cmp(current_time, tp->abs_deadline) > 0) {
			tp->dmiss++;
			return 1;
		}

		return 0;
}

//-----------------------------------------------------------------------------
//	DEBUG
//-----------------------------------------------------------------------------
void 	print_header()
{
		printf("x,vx,y,vy,theta,vtheta,xe,vxe,ye,vye,thetae,vthetae\n");
}

void 	print_state(const state* s, const state* s1)
{
		printf("%f,", s->x);
		printf("%f,", s->vx);
		printf("%f,", s->y);
		printf("%f,", s->vy);
		printf("%f,", s->theta);
		printf("%f,", s->vtheta);
		printf("%f,", s1->x);
		printf("%f,", s1->vx);
		printf("%f,", s1->y);
		printf("%f,", s1->vy);
		printf("%f,", s1->theta);
		printf("%f", s1->vtheta);
		printf("\n");
}

void 	print_state_1(const state* s)
{
		printf("%f,", s->x);
		printf("%f,", s->vx);
		printf("%f,", s->y);
		printf("%f,", s->vy);
		printf("%f,", s->theta);
		printf("%f,", s->vtheta);
		printf("\n");
}

void	print_u(float u[2])
{
		printf("%f, %f\n",u[0], u[1]);
}

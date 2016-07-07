//------------------------------------------------------------------------------
//	UTILS.C
//------------------------------------------------------------------------------

#include <pthread.h>
#include <error.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"
#include "simulator.h"

//------------------------------------------------------------------------------
//	CONSTANTS
//------------------------------------------------------------------------------
#define NSUM 1000	//number of RV used in gaussian random generator

//------------------------------------------------------------------------------
//	GLOABAL VARIABLE DEFINITIONS
//------------------------------------------------------------------------------
task_par 		regulator_tp[MAX_QUADROTORS];
pthread_t 		regulator_tid[MAX_QUADROTORS];
task_par 		guidance_tp[MAX_QUADROTORS];
pthread_t		guidance_tid[MAX_QUADROTORS];
task_par		gui_tp[1];
pthread_t		gui_tid[1];
task_par		user_tp[1];
pthread_t		user_tid[1];

// Mutex required for mutual exclusion
pthread_mutex_t guidance_mutex[MAX_QUADROTORS];
pthread_mutex_t dynamics_mutex[MAX_QUADROTORS];
pthread_mutex_t kalman_mutex[MAX_QUADROTORS];
pthread_mutex_t force_mutex[MAX_QUADROTORS];
pthread_mutex_t desired_traj_mutex[MAX_QUADROTORS];
pthread_mutex_t	guid_switches_mutex[MAX_QUADROTORS];
pthread_mutex_t	selected_quad_mutex;

//------------------------------------------------------------------------------
//	MATRIX HANDLING FUNCTION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function mat_zero
//	sets to zero all the elements of the given matrix
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
//	and puts the result in 'dst'
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
//	and puts the result in 'dst'
//------------------------------------------------------------------------------
void	mat_sum(int n, int m, float op1[][m], float op2[][m], float dst[][m])
{
		mat_sum_scalar(n, m, op1, op2, dst, 1);
}

//------------------------------------------------------------------------------
//	Function mat sub
//	evaluates the difference of two matrices 'op1' and 'op2'
//	and puts the result in 'dst'
//------------------------------------------------------------------------------
void	mat_sub(int n, int m, float op1[][m], float op2[][m], float dst[][m])
{
		mat_sum_scalar(n, m, op1, op2, dst, -1);
}

//------------------------------------------------------------------------------
//	Function mat_mul
//	evaluates the product of two matrices 'op1' and 'op2'
//	and puts the result in 'dst'
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
//	and puts the result in 'dst'
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
//	and puts the result in 'dst'
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
//	RANDOM NUMBERS GENERATION FUNCTIONS
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

//-----------------------------------------------------------------------------
//	MATH FUNCTIONS
//-----------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function modulo
//	returns the 'a' modulo 'b'
//------------------------------------------------------------------------------
int		modulo(int a, int b)
{
int		r;

		r = a % b;
    	return r < 0 ? r + b : r;
}

//------------------------------------------------------------------------------
//	TIMESPEC FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	Function time_to_ms
//	returns the time represented by the timespec time in milliseconds
//------------------------------------------------------------------------------
float	time_to_ms(struct timespec *time)
{
		return time->tv_sec * 1000 + (float)time->tv_nsec / 1000000;
}

//------------------------------------------------------------------------------
//	Function time_zero
//	set a timespec so that it represents the time zero
//------------------------------------------------------------------------------
static
void	time_zero(struct timespec *time)
{
		time->tv_sec = 0;
		time->tv_nsec = 0;
}

//------------------------------------------------------------------------------
//	Function time_copy
//	copies a source timespec 't_source' in a destination timespec pointed by
//	't_dest'
//------------------------------------------------------------------------------
static
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
static
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
//	Function time_diff
//	evaluates the difference between two time variables represented by
//	two timespec 't1' and 't2'
//	returns the difference as a timespec
//------------------------------------------------------------------------------
static
struct timespec	time_diff(struct timespec* t1, struct timespec* t2)
{
struct timespec	diff;

				diff.tv_sec = t1->tv_sec - t2->tv_sec;
				diff.tv_nsec = t1->tv_nsec - t2->tv_nsec;

				if (diff.tv_nsec < 0) {
					diff.tv_sec -= 1;
					diff.tv_nsec += 1000000000;
				}

				return diff;
}

//------------------------------------------------------------------------------
//	Function time_cmp
//	compares two time variables represented by two timespec 't1' and 't2'
//	and returns 0 if they are equal, 1 if 't1' > 't2', -1 if 't1' < 't2'
//------------------------------------------------------------------------------
static
int		time_cmp(struct timespec* t1, struct timespec* t2)
{
		if(t1->tv_sec > t2->tv_sec)
			return 1;
		if(t1->tv_sec < t2->tv_sec)
			return -1;
		if(t1->tv_nsec > t2->tv_nsec)
			return 1;
		if(t1->tv_nsec < t2->tv_nsec)
			return -1;
		return 0;
}

//-----------------------------------------------------------------------------
//	THREAD MANAGEMENT FUNCTIONS
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
			pthread_mutex_init(&desired_traj_mutex[i], NULL);
			pthread_mutex_init(&guid_switches_mutex[i], NULL);
			pthread_mutex_init(&(regulator_tp[i].mutex), NULL);
			pthread_mutex_init(&(guidance_tp[i].mutex), NULL);
		 }
            pthread_mutex_init(&selected_quad_mutex, NULL);
			pthread_mutex_init(&(gui_tp[0].mutex), NULL);
			pthread_mutex_init(&(user_tp[0].mutex), NULL);
}

//-----------------------------------------------------------------------------
//	Function mutex_destroy
//	destroy all the mutex required in the application
//-----------------------------------------------------------------------------
void	mutex_destroy()
{
int		i;

		 for(i = 0; i < MAX_QUADROTORS; i++) {
			pthread_mutex_destroy(&guidance_mutex[i]);
			pthread_mutex_destroy(&dynamics_mutex[i]);
			pthread_mutex_destroy(&kalman_mutex[i]);
			pthread_mutex_destroy(&force_mutex[i]);
			pthread_mutex_destroy(&desired_traj_mutex[i]);
			pthread_mutex_destroy(&guid_switches_mutex[i]);
			pthread_mutex_destroy(&(regulator_tp[i].mutex));
			pthread_mutex_destroy(&(guidance_tp[i].mutex));

		 }
            pthread_mutex_destroy(&selected_quad_mutex);
			pthread_mutex_destroy(&(gui_tp[0].mutex));
			pthread_mutex_destroy(&(user_tp[0].mutex));
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
static
void	update_activation_time(struct task_par* tp)
{
		time_add_delta(&(tp->activation_time), tp->period);
}

//-----------------------------------------------------------------------------
//	Function update_abs_deadline
//	updates the absolute deadline
//-----------------------------------------------------------------------------
static
void	update_abs_deadline(struct task_par* tp)
{
		time_add_delta(&(tp->abs_deadline), tp->period);
}

//-----------------------------------------------------------------------------
//	Function wait_for_period
//	suspends the thread until the next activation
//-----------------------------------------------------------------------------
static
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
static
int		deadline_miss(struct task_par* tp)
{
		if(time_cmp(&(tp->finish_time), &(tp->abs_deadline)) > 0) {
			pthread_mutex_lock(&(tp->mutex));
			tp->dmiss++;
			pthread_mutex_unlock(&(tp->mutex));
			return 1;
		}

		return 0;
}

//-----------------------------------------------------------------------------
//	Function set_start_time
//	set the start time of the task
//-----------------------------------------------------------------------------
void	set_start_time(struct task_par* tp)
{
struct timespec time;

		clock_gettime(CLOCK_MONOTONIC, &time);
		time_copy(time, &(tp->start_time));
}

//-----------------------------------------------------------------------------
//	Function set_finish_time
//	set the start time of the task
//-----------------------------------------------------------------------------
static
void	set_finish_time(struct task_par* tp)
{
struct timespec time;

		clock_gettime(CLOCK_MONOTONIC, &time);
		time_copy(time, &(tp->finish_time));
}

//-----------------------------------------------------------------------------
//	Function zero_wcet
//	set the wcet to zero
//-----------------------------------------------------------------------------
void	zero_wcet(struct task_par* tp)
{
		time_zero(&(tp->wcet));
}

//-----------------------------------------------------------------------------
//	Function update_wcet
//	update the worst case execution time of the task
//-----------------------------------------------------------------------------
static
void	update_wcet(struct task_par* tp)
{
struct timespec diff;

		diff = time_diff(&(tp->finish_time), &(tp->start_time));

		pthread_mutex_lock(&(tp->mutex));

		if (time_cmp(&diff, &(tp->wcet))== 1)
			time_copy(diff, &(tp->wcet));

		pthread_mutex_unlock(&(tp->mutex));
}

//-----------------------------------------------------------------------------
//	Function aperiodic_wait
//	suspends the thread until the next activation
//-----------------------------------------------------------------------------
void	aperiodic_wait(struct task_par* tp)
{
struct timespec	time;

		clock_gettime(CLOCK_MONOTONIC, &time);
		time_add_delta(&time, tp->period);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, NULL);
}

//-----------------------------------------------------------------------------
//	Function thread_loop_end
//	updates thread parameters and waits for the next attivation
//-----------------------------------------------------------------------------
void	thread_loop_end(struct task_par* tp)
{
		set_finish_time(tp);
		update_wcet(tp);
		deadline_miss(tp);
		wait_for_period(tp);
		update_activation_time(tp);
		update_abs_deadline(tp);
}

//------------------------------------------------------------------------------
//	Function create_task
// 	creates a task given the task name, parameters, id, body,
//	period, relative deadline and the no. of replica required;
//	return 0 if success, one of the ERRNO otherwise
//------------------------------------------------------------------------------
int		create_task(char* task_name, task_par tp[], pthread_t tid[],\
					void*(*task_body)(void*), int period, int deadline, int replica)
{
int		i;
int 	error;

		for(i = 0; i < replica; i++) {
			tp[i].id = i;
			tp[i].period = period;
			tp[i].deadline = deadline;
			tp[i].dmiss = 0;
			strcpy(tp[i].task_name, task_name);
			zero_wcet(&(tp[i]));

			error = pthread_create(&tid[i], NULL, task_body, &tp[i]);
			if (error != 0) {
				perror("(pthread_create) Error:");
				return error;
			}
		}
		return 0;
}

//------------------------------------------------------------------------------
//	Function wait_for_task_end
//------------------------------------------------------------------------------
int		wait_for_tasks_end()
{
int 	i, error;

		for(i = 0; i < MAX_QUADROTORS; i++) {
			error = pthread_join(regulator_tid[i], NULL);
			if (error != 0) {
				perror("(pthread_join) Error:");
				return error;
			}

			error = pthread_join(guidance_tid[i], NULL);
			if (error != 0) {
				perror("(pthread_join) Error:");
				return error;
			}
		}
		error = pthread_join(gui_tid[0], NULL);
		if (error != 0) {
			perror("(pthread_join) Error:");
		}
		error = pthread_join(user_tid[0], NULL);
		if (error != 0) {
			perror("(pthread_user) Error:");
		}

		return 0;
}

//------------------------------------------------------------------------------
//	Function cancel_thread_all
//------------------------------------------------------------------------------
void	cancel_thread_all()
{
int		i, retval;

		pthread_cancel(gui_tid[0]);

		for(i = 0; i < MAX_QUADROTORS; i++) {
			pthread_cancel(guidance_tid[i]);
			pthread_cancel(regulator_tid[i]);
		}

		pthread_exit((void*)&retval);
}

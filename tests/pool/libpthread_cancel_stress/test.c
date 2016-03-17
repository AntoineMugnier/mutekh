/*
* Copyright (c) 2004, Bull S.A..  All rights reserved.
* Created by: Sebastien Decugis

* This program is free software; you can redistribute it and/or modify it
* under the terms of version 2 of the GNU General Public License as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it would be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write the Free Software Foundation, Inc., 59
* Temple Place - Suite 330, Boston MA 02111-1307, USA.


* This stress test aims to test the following assertion:

*  Heavy cancelation does not break the system or the user application.

* The steps are:
* Create some threads which:
*  Create a thread.
*  Cancel this thread, as it terminates.
*  Check the return value.

*/


/* We are testing conformance to IEEE Std 1003.1, 2003 Edition */
#define _POSIX_C_SOURCE 200112L

/********************************************************************************************/
/****************************** standard includes *****************************************/
/********************************************************************************************/
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <errno.h>

/********************************************************************************************/
/******************************   Test framework   *****************************************/
/********************************************************************************************/
#include "testfrmw.h"
/* This header is responsible for defining the following macros:
 * UNRESOLVED(ret, descr);  
 *    where descr is a description of the error and ret is an int (error code for example)
 * FAILED(descr);
 *    where descr is a short text saying why the test has failed.
 * PASSED();
 *    No parameter.
 * 
 * Both three macros shall terminate the calling process.
 * The testcase shall not terminate in any other maneer.
 * 
 * The other file defines the functions
 * void output_init()
 * void output(char * string, ...)
 * 
 * Those may be used to output information.
 */

/********************************************************************************************/
/********************************** Configuration ******************************************/
/********************************************************************************************/
#ifndef VERBOSE
#define VERBOSE 1
#endif

#define NTHREADS 32

/********************************************************************************************/
/***********************************    Test cases  *****************************************/
/********************************************************************************************/

test_cpu_barrier_t cpu_barrier;
long canceled, ended;

/* The canceled thread */
void * th( void * arg )
{
	int ret = 0;
	ret = pthread_barrier_wait( arg );

	if ( ( ret != 0 ) && ( ret != PTHREAD_BARRIER_SERIAL_THREAD ) )
	{
		UNRESOLVED( ret, "Failed to wait for the barrier" );
	}

	while (rand() % 4)
	{
		TEST_RANDOM_WAIT(4);
		pthread_yield();
	}

	return NULL;
}


/* Thread function */
void * threaded( void * arg )
{
	int ret = 0;
	pthread_t child;

#if CONFIG_CPU_MAXCOUNT > NTHREADS
# error Not all processors are used in test
#endif

	/* ensure all processors are in use */
	test_cpu_barrier_wait(&cpu_barrier);

	output( "[parent] Processor barrier done\n" );

	/* Initialize the barrier */
	ret = pthread_barrier_init( arg, NULL, 2 );

	if ( ret != 0 )
	{
		UNRESOLVED( ret, "Failed to initialize a barrier" );
	}


	while ( TEST_CONTINUE )
	{
		pthread_attr_t a;

		pthread_attr_init(&a);
#ifdef CONFIG_MUTEK_CONTEXT_SCHED_STATIC
		pthread_attr_affinity(&a, rand() % CONFIG_CPU_MAXCOUNT);
#endif

		/* Create the thread */
		ret = pthread_create( &child, &a, th, arg );

		pthread_attr_destroy(&a);

		if ( ret != 0 )
		{
			UNRESOLVED( ret, "Thread creation failed" );
		}

		/* Synchronize */
		ret = pthread_barrier_wait( arg );

		if ( ( ret != 0 ) && ( ret != PTHREAD_BARRIER_SERIAL_THREAD ) )
		{
			UNRESOLVED( ret, "Failed to wait for the barrier" );
		}

		TEST_RANDOM_WAIT(1);

		/* Cancel the thread */
		ret = pthread_cancel( child );

		if ( ret == 0 )
			canceled++;
		else
			ended++;

		/* Join the thread */
		ret = pthread_join( child, NULL );

		if ( ret != 0 )
		{
			UNRESOLVED( ret, "Unable to join the child" );
		}

	}


	/* Destroy the barrier */
	ret = pthread_barrier_destroy( arg );

	if ( ret != 0 )
	{
		UNRESOLVED( ret, "Failed to destroy a barrier" );
	}

	return NULL;
}

/* Main function */
int main ( int argc, char *argv[] )
{
        int ret = 0, i, iter;

	pthread_t th[ NTHREADS ];
	pthread_barrier_t b[ NTHREADS ];

	TEST_START;

	test_cpu_barrier_init(&cpu_barrier);

        for ( iter = 0; iter < 4; iter++){
	        for ( i = 0; i < NTHREADS; i++ )
		{
			pthread_attr_t a;

			pthread_attr_init(&a);
#ifdef CONFIG_MUTEK_CONTEXT_SCHED_STATIC
			pthread_attr_affinity(&a, i % CONFIG_CPU_MAXCOUNT);
#endif

			ret = pthread_create( &th[ i ], &a, threaded, &b[ i ] );

			if ( ret != 0 )
				{
					UNRESOLVED( ret, "Failed to create a thread" );
				}

			pthread_attr_destroy(&a);
		}

	        output( "[parent] All threads were started\n" );

	        /* Then join */
	        for ( i = 0; i < NTHREADS; i++ )
		{
			ret = pthread_join( th[ i ], NULL );

			if ( ret != 0 )
				{
					UNRESOLVED( ret, "Failed to join a thread" );
				}
		}
	
	        /* We've been asked to stop */

	        output( " - %llu threads canceled\n", canceled );
	        output( " - %llu threads ended\n", ended );

	        if ( !canceled || !ended )
		        UNRESOLVED( ret, "Not all cases were tested successfully" );

        }
	PASSED;
}


/*
 * This file is distributed under the terms of the GNU General Public
 * License.
 * 
 * Copyright (c) UPMC / Lip6
 *     2005-2008, Nicolas Pouillon, <nipo@ssji.net>
 */
#include <pthread.h>
#include <string.h>
#include <assert.h>

#include <mwmr.h>

void mwmr_read( mwmr_t fifo, void *mem, size_t len )
{
    unsigned int got = 0;
    uint8_t *ptr = (uint8_t *)mem;

    assert ( len % fifo->width == 0 );

    pthread_mutex_lock( &(fifo->lock) );
    while ( got < len ) {
        while ( ! fifo->usage ) {
            pthread_cond_wait( &(fifo->nempty), &(fifo->lock) );
        }
        memcpy( ptr, fifo->rptr, fifo->width );
        fifo->rptr += fifo->width;
        if ( fifo->rptr == fifo->end )
            fifo->rptr = fifo->buffer;
        fifo->usage -= fifo->width;
        pthread_cond_signal( &(fifo->nfull) );
        got += fifo->width;
        ptr += fifo->width;
    }
    pthread_mutex_unlock( &(fifo->lock) );
    pthread_cond_signal( &(fifo->nfull) );
    pthread_yield();
}

void mwmr_write( mwmr_t fifo, void *mem, size_t len )
{
    unsigned int put = 0;
    uint8_t *ptr = (uint8_t *)mem;

    assert ( len % fifo->width == 0 );

    pthread_mutex_lock( &(fifo->lock) );
    while ( put < len ) {
        while ( fifo->usage == fifo->depth ) {
            pthread_cond_wait( &(fifo->nfull), &(fifo->lock) );
        }
        memcpy( fifo->wptr, ptr, fifo->width );
        fifo->wptr += fifo->width;
        if ( fifo->wptr == fifo->end )
            fifo->wptr = fifo->buffer;
        fifo->usage += fifo->width;
        pthread_cond_signal( &(fifo->nempty) );
        put += fifo->width;
        ptr += fifo->width;
    }
    pthread_mutex_unlock( &(fifo->lock) );
    pthread_cond_signal( &(fifo->nempty) );

    pthread_yield();
}

ssize_t mwmr_try_read( mwmr_t fifo, void *mem, size_t len )
{
    unsigned int got = 0;
    uint8_t *ptr = (uint8_t *)mem;

    assert ( len % fifo->width == 0 );

    if ( pthread_mutex_trylock( &(fifo->lock) ) ) {
        return 0;
    }
    
    while ( got < len ) {
        if ( ! fifo->usage ) {
            pthread_mutex_unlock( &(fifo->lock) );
            pthread_cond_signal( &(fifo->nfull) );
            return got;
        }
        memcpy( ptr, fifo->rptr, fifo->width );
        fifo->rptr += fifo->width;
        if ( fifo->rptr == fifo->end )
            fifo->rptr = fifo->buffer;
        fifo->usage -= fifo->width;
        got += fifo->width;
        ptr += fifo->width;
    }
    pthread_mutex_unlock( &(fifo->lock) );
    pthread_cond_signal( &(fifo->nfull) );

    pthread_yield();

    return got;
}

ssize_t mwmr_try_write( mwmr_t fifo, void *mem, size_t len )
{
    unsigned int put = 0;
    uint8_t *ptr = (uint8_t *)mem;

    assert( len % fifo->width == 0 );

    if ( pthread_mutex_trylock( &(fifo->lock) ) ) {
        return 0;
    }

    while ( put < len ) {
        if ( fifo->usage == fifo->depth ) {
            pthread_mutex_unlock( &(fifo->lock) );
            pthread_cond_signal( &(fifo->nempty) );
            return put;
        }
        memcpy( fifo->wptr, ptr, fifo->width );
        fifo->wptr += fifo->width;
        if ( fifo->wptr == fifo->end )
            fifo->wptr = fifo->buffer;
        fifo->usage += fifo->width;
        put += fifo->width;
        ptr += fifo->width;
    }
    pthread_mutex_unlock( &(fifo->lock) );
    pthread_cond_signal( &(fifo->nempty) );

    pthread_yield();

    return put;
}


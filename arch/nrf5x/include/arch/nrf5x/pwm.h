/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#ifndef ARCH_NRF_PWM_H_
#define ARCH_NRF_PWM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_pwm_task {
    NRF_PWM_STOP = 1,
    NRF_PWM_SEQSTART0 = 2,
    NRF_PWM_SEQSTART1 = 3,
    NRF_PWM_NEXTSTEP = 4,
};

enum nrf5x_pwm_event {
    NRF_PWM_STOPPED = 1,
    NRF_PWM_SEQSTARTED0 = 2,
    NRF_PWM_SEQSTARTED1 = 3,
    NRF_PWM_SEQEND0 = 4,
    NRF_PWM_SEQEND1 = 5,
    NRF_PWM_PWMPERIODEN = 6,
    NRF_PWM_LOOPSDONE = 7,
};

enum nrf5x_pwm_short {
    NRF_PWM_SEQEND0_STOP,
    NRF_PWM_SEQEND1_STOP,
    NRF_PWM_LOOPSDONE_SEQSTART0,
    NRF_PWM_LOOPSDONE_SEQSTART1,
    NRF_PWM_LOOPSDONE_STOP,
};

enum nrf5x_pwm_register {
    NRF_PWM_ENABLE = 64,
    NRF_PWM_MODE,
    NRF_PWM_COUNTERTOP,
    NRF_PWM_PRESCALER,
    NRF_PWM_DECODER,
    NRF_PWM_LOOP,

    NRF_PWM_SEQ0_PTR = 72,
    NRF_PWM_SEQ0_CNT,
    NRF_PWM_SEQ0_REFRESH,
    NRF_PWM_SEQ0_ENDDELAY,

    NRF_PWM_SEQ1_PTR = 80,
    NRF_PWM_SEQ1_CNT,
    NRF_PWM_SEQ1_REFRESH,
    NRF_PWM_SEQ1_ENDDELAY,

    NRF_PWM_PSEL_OUT0 = 88,
    NRF_PWM_PSEL_OUT1,
    NRF_PWM_PSEL_OUT2,
    NRF_PWM_PSEL_OUT3,
};

#define NRF_PWM_ENABLE_ENABLED 1

#define NRF_PWM_MODE_UPDOWN_MASK 1
#define NRF_PWM_MODE_UPDOWN_UP 0
#define NRF_PWM_MODE_UPDOWN_DOWN 1

#define NRF_PWM_PRESCALER_POW2(x) (x)

#define NRF_PWM_DECODER_LOAD_MASK 0x7
#define NRF_PWM_DECODER_LOAD_COMMON 0
#define NRF_PWM_DECODER_LOAD_GROUPED 1
#define NRF_PWM_DECODER_LOAD_INDIVIDUAL 2
#define NRF_PWM_DECODER_LOAD_WAVEFORM 3
#define NRF_PWM_DECODER_MODE_MASK 0x10
#define NRF_PWM_DECODER_MODE_REFRESHCOUNT 0
#define NRF_PWM_DECODER_MODE_NEXTSTEP 0x10

#endif

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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef PCAL6408A_H_
#define PCAL6408A_H_

#define PCAL6408A_INPUT_PORT                0x00
#define PCAL6408A_OUTPUT_PORT               0x01
#define PCAL6408A_POLARITY_INVERSION        0x02
#define PCAL6408A_DIR_IN                    0x03
#define PCAL6408A_OUTPUT_DRIVE_STRENGTH_0   0x40
#define PCAL6408A_OUTPUT_DRIVE_STRENGTH_1   0x41
#define PCAL6408A_INPUT_LATCH_EN            0x42
#define PCAL6408A_PULL_ENABLE               0x43
#define PCAL6408A_PULL_UP                   0x44
#define PCAL6408A_IRQ_DISABLE               0x45
#define PCAL6408A_INTERRUPT_STATUS          0x46
#define PCAL6408A_OUTPUT_PORT_CONFIGURATION 0x4F
#define  PCAL6408A_OUTPUT_PORT_OPENDRAIN     0x01

#endif

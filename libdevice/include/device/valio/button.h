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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a button
*/

#ifndef LIBDEVICE_VALIO_BUTTON_H_
#define LIBDEVICE_VALIO_BUTTON_H_

#include <device/class/valio.h>

enum valio_button_state {
  VALIO_BUTTON_PUSHED,
  VALIO_BUTTON_RELEASED
};

enum valio_button_att {
    /** This attribute is used only for @tt DEVICE_VALIO_WAIT_EVENT request type.
        When used, the request callback will be called when button is toggled,
        pushed or released. Request must returns the time since last event on
        button in the @tt timestamp field of data. If no measure of time is possible
        in driver 0 must be returned.
    **/
    VALIO_BUTTON_TOGGLE = CONFIG_DEVICE_VALIO_BUTTON_ATTRIBUTE_FIRST,
    VALIO_BUTTON_PUSH,
    VALIO_BUTTON_RELEASE,
    VALIO_BUTTON_SUSTAINED_PUSH,
    VALIO_BUTTON_DELAYED_PUSH,
};

typedef void (*valio_push_button_event_t)(struct dev_valio_rq_s *rq);

/* Return structure for @tt DEVICE_VALIO_WAIT_EVENT request type */
struct valio_button_update_s
{
  valio_push_button_event_t pb_event; // Callback for sustain event
  union {
    uint32_t delay; // delay value for sustained push event
    uint32_t timestamp; // event timestamp
  };
};

/* Return structure for @tt DEVICE_VALIO_READ request type */
struct valio_button_read_s
{
  bool_t state;
};

#endif

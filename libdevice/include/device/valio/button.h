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
   @module{Devices support library}
   @short Value IO interface for a button
*/

#ifndef LIBDEVICE_VALIO_BUTTON_H_
#define LIBDEVICE_VALIO_BUTTON_H_

enum valio_button_state {
  VALIO_BUTTON_PUSHED,
  VALIO_BUTTON_RELEASED
};

enum valio_button_att {
    /** This attribute is used only for @tt DEVICE_VALIO_WAIT_UPDATE request type.
        When used, the request callback will be called when button is toggled,
        pushed or released. Request must returns the time since last event on
        button in the @tt timestamp field of data. If no measure of time is possible
        in driver 0 must be returned.
    **/
    VALIO_BUTTON_TOGGLE,
    VALIO_BUTTON_PUSH,
    VALIO_BUTTON_RELEASE,
};

/* Return structure for @tt DEVICE_VALIO_WAIT_UPDATE request type */ 
struct valio_button_update_s
{
  uint16_t timestamp;
};

/* Return structure for @tt DEVICE_VALIO_READ request type */ 
struct valio_button_read_s
{
  bool_t state;
};

#endif

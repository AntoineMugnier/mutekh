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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_GATT_H
#define BLE_GATT_H

enum ble_gatt_properties_e {
	BLE_GATT_BROADCAST	= 0x01,
	BLE_GATT_READ		= 0x02,
	BLE_GATT_WRITE_WO_RSP	= 0x04,
	BLE_GATT_WRITE		= 0x08,
	BLE_GATT_NOTIFY		= 0x10,
	BLE_GATT_INDICATE	= 0x20,
	BLE_GATT_AUTH_WRITE	= 0x40,
	BLE_GATT_EXTENDED	= 0x80,
};

enum ble_gatt_extended_properties_e {
	BLE_GATT_RELIABLE_WRITE	= 0x01,
	BLE_GATT_WRITABLE_AUX	= 0x02,
};

enum ble_gatt_notification_mode_e {
	BLE_GATT_CCCD_NOTIFICATION	= 0x01,
	BLE_GATT_CCCD_INDICATION	= 0x02,
};

enum ble_gatt_handle_pair_type_e {
	BLE_GATT_NONE = 0,
	BLE_GATT_HANDLE_UUID16 = 0x01,
	BLE_GATT_HANDLE_UUID128 = 0x02,
};

#endif
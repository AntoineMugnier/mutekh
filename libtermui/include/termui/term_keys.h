/*
    This file is part of libtermui.

    libtermui is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libtermui is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libtermui.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2006, Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#ifndef TERM_KEYS_H_
#define TERM_KEYS_H_

#define TERM_KEY_NUL		0x00000000 /* CTRL space */
#define TERM_KEY_SOH		0x00000001 /* CTRL a */
#define TERM_KEY_STX		0x00000002 /* CTRL b */
#define TERM_KEY_ETX		0x00000003 /* CTRL c */
#define TERM_KEY_EOT		0x00000004 /* CTRL d */
#define TERM_KEY_ENQ		0x00000005 /* CTRL e */
#define TERM_KEY_ACK		0x00000006 /* CTRL f */
#define TERM_KEY_BEL		0x00000007 /* CTRL g */
#define TERM_KEY_BS		0x00000008 /* CTRL h */
#define TERM_KEY_HT		0x00000009 /* CTRL i */
#define TERM_KEY_LF		0x0000000a /* CTRL j */
#define TERM_KEY_VT		0x0000000b /* CTRL k */
#define TERM_KEY_FF		0x0000000c /* CTRL l */
#define TERM_KEY_RETURN		0x0000000d /* CTRL m */
#define TERM_KEY_SO		0x0000000e /* CTRL n */
#define TERM_KEY_SI		0x0000000f /* CTRL o */
#define TERM_KEY_DLE		0x00000010 /* CTRL p */
#define TERM_KEY_DC1		0x00000011 /* CTRL q */
#define TERM_KEY_DC2		0x00000012 /* CTRL r */
#define TERM_KEY_DC3		0x00000013 /* CTRL s */
#define TERM_KEY_DC4		0x00000014 /* CTRL t */
#define TERM_KEY_NAK		0x00000015 /* CTRL u */
#define TERM_KEY_SYN		0x00000016 /* CTRL v */
#define TERM_KEY_ETB		0x00000017 /* CTRL w */
#define TERM_KEY_CAN		0x00000018 /* CTRL x */
#define TERM_KEY_EM		0x00000019 /* CTRL y */
#define TERM_KEY_SUB		0x0000001a /* CTRL z */
#define TERM_KEY_ESC		0x0000001b
#define TERM_KEY_FS		0x0000001c
#define TERM_KEY_GS		0x0000001d
#define TERM_KEY_RS		0x0000001e
#define TERM_KEY_US		0x0000001f

#define TERM_KEY_SPACE		0x00000020

#define TERM_KEY_DELETE		0x0000007f

#define TERM_KEY_META(n)	((n) | 0x00000100)

#define TERM_KEY_FCN(n)		((n) + 0x00000200)
#define TERM_KEY_UP		0x00000214
#define TERM_KEY_DOWN		0x00000215
#define TERM_KEY_LEFT		0x00000216
#define TERM_KEY_RIGHT		0x00000217
#define TERM_KEY_INSERT		0x00000218
#define TERM_KEY_HOME		0x00000219
#define TERM_KEY_END		0x00000220
#define TERM_KEY_PGUP		0x00000221
#define TERM_KEY_PGDN		0x00000222
#define TERM_KEY_UNTAB		0x00000223
#define TERM_KEY_REMOVE		0x00000224

#define TERM_MAX_KEY		0x00000225

#endif


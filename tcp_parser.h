/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



*/

#ifndef TCP_PARSER_H
#define TCP_PARSER_H

typedef struct
{
	// Where to write when receiving. This field is ignored for tx. 
	// Remember to use packed structs
	void* p_data; 
	// Message ID
	uint8_t mid;
	// Number of bytes of data expected / sent
	int size;
	// Zero-terminated string: Interpretation of the bytes:
	char types[32]; 
	/*
		'b' int8_t
		'B' uint8_t
		's' int16_t
		'S' uint16_t
		'i' int32_t
		'I' uint32_t
		'l' int64_t
		'L' uint64_t
	*/
	int ret;
} tcp_message_t;


#define TCP_CR_MAINTENANCE_MID    62
typedef struct __attribute__ ((packed))
{
	int32_t magic;
	int32_t retval;
} tcp_cr_maintenance_t;

extern tcp_cr_maintenance_t   msg_cr_maintenance;

#define TCP_RC_PICTURE_MID	    142

int tcp_parser(int sock);

int tcp_send_msg(tcp_message_t* msg_type, void* msg);

void tcp_send_picture(int16_t id, uint8_t bytes_per_pixel, int xs, int ys, uint8_t *pict);


#endif

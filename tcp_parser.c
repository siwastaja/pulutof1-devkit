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


	TCP communication functions.
	Robot is the "server", client can be a client software directly, or a relaying server.
	

*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "tcp_comm.h"
#include "tcp_parser.h"

tcp_cr_maintenance_t msg_cr_maintenance;
tcp_message_t msgmeta_cr_maintenance =
{
	&msg_cr_maintenance,
	TCP_CR_MAINTENANCE_MID,
	8, "ii"
};

#define NUM_CR_MSGS 1
tcp_message_t* CR_MSGS[NUM_CR_MSGS] =
{
	&msgmeta_cr_maintenance
};

#define I32TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>24)&0xff; b_[(s_)+1] = ((i_)>>16)&0xff; b_[(s_)+2] = ((i_)>>8)&0xff; b_[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>8)&0xff; b_[(s_)+1] = ((i_)>>0)&0xff; }

void tcp_send_picture(int16_t id, uint8_t bytes_per_pixel, int xs, int ys, uint8_t *pict)
{
	if(xs < 1 || ys < 1 || xs > 10000 || ys > 10000 || bytes_per_pixel < 1 || bytes_per_pixel > 4 )
	{
		printf("ERROR: tcp_send_picture: invalid params\n.");
		return;
	}

	int size=3+2+1+2+2+(xs*ys)*bytes_per_pixel;
	uint8_t *buf = malloc(size);

	if(!buf)
	{
		printf("ERROR: Out of memory in tcp_send_picture\n");
		return;
	}

	buf[0] = TCP_RC_PICTURE_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;

	I16TOBUF(id, buf, 3);
	buf[5] = bytes_per_pixel;
	I16TOBUF(xs, buf, 6);
	I16TOBUF(ys, buf, 8);

	memcpy(&buf[10], pict, bytes_per_pixel*xs*ys);

	tcp_send(buf, size);

	free(buf);
}

int tcp_send_msg(tcp_message_t* msg_type, void* msg)
{
	static uint8_t sendbuf[65536];
	sendbuf[0] = msg_type->mid;
	sendbuf[1] = (msg_type->size>>8)&0xff;
	sendbuf[2] = msg_type->size&0xff;

	uint8_t* p_dest = sendbuf+3;
	uint8_t* p_src = msg;

	for(int field=0;;field++)
	{
		switch(msg_type->types[field])
		{
			case 'b':
			case 'B':
				*(p_dest++) = *(p_src++);
			break;

			case 's':
			case 'S':
			{
				*(p_dest++) = (((*((uint16_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint16_t*)p_src)) )>>0)&0xff;
				p_src+=2;
			}
			break;

			case 'i':
			case 'I':
			{
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>24)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>16)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>0)&0xff;
				p_src+=4;
			}
			break;

			case 'l':
			case 'L':
			{
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>56)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>48)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>40)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>32)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>24)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>16)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>0)&0xff;
				p_src+=8;
			}
			break;

			case 0:
				goto PARSE_END;

			default:
				fprintf(stderr, "ERROR: parse type string has invalid character 0x%02x\n", msg_type->types[field]);
				return -2;
		}
	}

	PARSE_END: ;

	//printf("Sending tcp, size=%d\n", msg_type->size+3);
	tcp_send(sendbuf, msg_type->size+3);

	return 0;
}

/*
Return value:
< 0: Error.
0: Message was not parsed (not fully received yet)
>0: Message ID of the parsed message.
*/

int tcp_parser(int sock)
{
	int ret;
	static int state = 0; // Num of bytes read
	static struct __attribute__ ((packed)) { uint8_t mid; uint8_t size_msb; uint8_t size_lsb;} header; // Stored header of incoming message
	static int bytes_left = 0;
	static tcp_message_t* msg = 0; // Once message is recognized, pointer to message type struct.
	static int unrecog = 0;

	static uint8_t buf[65536];
	static uint8_t* p_buf = buf;

	if(state < 3)
	{
		msg = 0;
		unrecog = 0;
		ret = read(sock, (uint8_t*)&(header.mid) + state, 3-state);
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
			state = 0;
			return -11;
		}
		else if(ret == 0)
		{
			fprintf(stderr, "Client closed connection\n");
			state = 0;
			return -10;
		}
		else
			state += ret;
	}

	if(state >= 3)
	{
		if(msg == 0 && unrecog == 0)
		{		
			for(int i=0; i<NUM_CR_MSGS; i++)
			{
				if(CR_MSGS[i]->mid == header.mid)
				{
					msg = CR_MSGS[i];
					break;
				}
			}

			int size_from_header = ((int)header.size_msb<<8) | (int)header.size_lsb;
			if(!msg)
			{
				fprintf(stderr, "WARN: Ignoring unrecognized message with msgid 0x%02x\n", header.mid);
				unrecog = 1;
			}
			else if(size_from_header != msg->size)
			{
				fprintf(stderr, "WARN: Ignoring message with msgid 0x%02x because of size mismatch (got:%u, expected:%u)\n",
					header.mid, size_from_header, msg->size);
				unrecog = 1;
			}
			bytes_left = size_from_header;
			p_buf = buf;
		}

		// Read as much data as the size field shows.
		if(bytes_left)
		{
			ret = read(sock, p_buf, bytes_left);
			if(ret < 0)
			{
				fprintf(stderr, "ERROR: TCP stream read error %d (%s), closing socket.\n", errno, strerror(errno));
				tcp_comm_close();
			}
			else if(ret == 0)
			{
				fprintf(stderr, "ERROR: TCP stream read() returned 0 bytes even when it shoudln't, closing socket.\n");
				tcp_comm_close();
			}
			else
			{
				bytes_left -= ret;
				p_buf += ret;
			}
		}

		if(bytes_left < 0)
		{
			fprintf(stderr, "ERROR: bytes_left < 0, closing socket.\n");
			tcp_comm_close();
			bytes_left = 0;
		}

		if(bytes_left == 0)
		{
			state = 0;

			if(!unrecog)
			{
				// Parse the message
				uint8_t* p_src = buf;
				void* p_dest = msg->p_data;

				if(p_dest == 0)
				{
					fprintf(stderr, "ERROR: message parsing destination pointer NULL\n");
					return -1;
				}

				for(int field=0;;field++)
				{
					switch(msg->types[field])
					{
						case 'b':
						case 'B':
							*((uint8_t*)p_dest) = *(p_src++);
							p_dest = ((uint8_t*)p_dest) + 1;
						break;

						case 's':
						case 'S':
						{
							uint16_t tmp  = ((uint16_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint16_t*)p_dest) = tmp;
							p_dest = ((uint16_t*)p_dest) + 1;
						}
						break;

						case 'i':
						case 'I':
						{
							uint32_t tmp  = ((uint32_t)(*(p_src++))<<24);
								 tmp |= ((uint32_t)(*(p_src++))<<16);
								 tmp |= ((uint32_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint32_t*)p_dest) = tmp;
							p_dest = ((uint32_t*)p_dest) + 1;
						}
						break;

						case 'l':
						case 'L':
						{
							uint64_t tmp  = ((uint64_t)(*(p_src++))<<56);
								 tmp |= ((uint64_t)(*(p_src++))<<48);
								 tmp |= ((uint64_t)(*(p_src++))<<40);
								 tmp |= ((uint64_t)(*(p_src++))<<32);
								 tmp |= ((uint64_t)(*(p_src++))<<24);
								 tmp |= ((uint64_t)(*(p_src++))<<16);
								 tmp |= ((uint64_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint64_t*)p_dest) = tmp;
							p_dest = ((uint64_t*)p_dest) + 1;
						}
						break;


						case 0:
							goto PARSE_END;

						default:
							fprintf(stderr, "ERROR: parse type string has invalid character 0x%02x\n", msg->types[field]);
							return -2;

					}
				}

				PARSE_END: ;
				return header.mid;
			}
		}

	}

	return 0;
}


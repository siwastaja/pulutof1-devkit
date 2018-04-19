/*
	PULUROBOT PULUTOF-DEVKIT development software for saving pointclouds

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

#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "datatypes.h"
#include "pulutof.h"

volatile int verbose_mode = 0;
volatile int send_raw_tof = -1;
volatile int send_pointcloud = 0; // 0 = off, 1 = relative to robot, 2 = relative to actual world coords

double subsec_timestamp()
{
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return (double)spec.tv_sec + (double)spec.tv_nsec/1.0e9;
}


void save_pointcloud(int n_points, xyz_t* cloud)
{
	static int pc_cnt = 0;
	char fname[256];
	snprintf(fname, 255, "cloud%05d.xyz", pc_cnt);
	printf("Saving pointcloud with %d samples to file %s.\n", n_points, fname);
	FILE* pc_csv = fopen(fname, "w");
	if(!pc_csv)
	{
		printf("Error opening file for write.\n");
	}
	else
	{
		for(int i=0; i < n_points; i++)
		{
			fprintf(pc_csv, "%d %d %d\n",cloud[i].x, -1*cloud[i].y, cloud[i].z);
		}
		fclose(pc_csv);
	}

	pc_cnt++;
	if(pc_cnt > 99999) pc_cnt = 0;
}



void request_tof_quit(void);

volatile int retval = 0;

void* main_thread()
{

	if(init_tcp_comm())
	{
		fprintf(stderr, "TCP communication initialization failed.\n");
		return NULL;
	}
	while(1)
	{
		// Calculate fd_set size (biggest fd+1)
		int fds_size = 0;
		if(tcp_listener_sock > fds_size) fds_size = tcp_listener_sock;
		if(tcp_client_sock > fds_size) fds_size = tcp_client_sock;
		if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
		fds_size+=1;

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(tcp_listener_sock, &fds);
		if(tcp_client_sock >= 0)
			FD_SET(tcp_client_sock, &fds);

		struct timeval select_time = {0, 200};

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return NULL;
		}

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'q')
			{
				retval = 0;
				break;
			}
			if(cmd == 'z')
			{
				pulutof_decr_dbg();
			}
			if(cmd == 'x')
			{
				pulutof_incr_dbg();
			}
			if(cmd == 'Z')
			{
				if(send_raw_tof >= 0) send_raw_tof--;
				printf("Sending raw tof from sensor %d\n", send_raw_tof);
			}
			if(cmd == 'X')
			{
				if(send_raw_tof < 3) send_raw_tof++;
				printf("Sending raw tof from sensor %d\n", send_raw_tof);
			}
			if(cmd >= '1' && cmd <= '4')
			{
				pulutof_cal_offset(cmd-'1');
			}
			if(cmd == 'p')
			{
				if(send_pointcloud == 0)
				{
					printf("INFO: Will send pointclouds relative to robot origin\n");
					send_pointcloud = 1;
				}
				else if(send_pointcloud == 1)
				{
					printf("INFO: Will send pointclouds relative to world origin\n");
					send_pointcloud = 2;
				}
				else
				{
					printf("INFO: Will stop sending pointclouds\n");
					send_pointcloud = 0;
				}
			}
		}

		if(tcp_client_sock >= 0 && FD_ISSET(tcp_client_sock, &fds))
		{
			int ret = handle_tcp_client();
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
		}

#ifdef PULUTOF1_GIVE_RAWS

		pulutof_frame_t* p_tof;
		if( (p_tof = get_pulutof_frame()) )
		{
			if(tcp_client_sock >= 0)
			{
#ifdef PULUTOF_EXTRA
				tcp_send_picture(p_tof->dbg_id, 2, 160, 60, p_tof->dbg);
#endif
				tcp_send_picture(100,           2, 160, 60, (uint8_t*)p_tof->depth);
#ifdef PULUTOF_EXTRA
				tcp_send_picture(110,           2, 160, 60, (uint8_t*)p_tof->uncorrected_depth);
#endif
			}

		}

#else
		tof3d_scan_t *p_tof;
		if( (p_tof = get_tof3d()) )
		{

			if(tcp_client_sock >= 0)
			{
				static int hmap_cnt = 0;
				hmap_cnt++;

				if(hmap_cnt >= 4)
				{
					tcp_send_hmap(TOF3D_HMAP_XSPOTS, TOF3D_HMAP_YSPOTS, p_tof->robot_pos.ang, p_tof->robot_pos.x, p_tof->robot_pos.y, TOF3D_HMAP_SPOT_SIZE, p_tof->objmap);

					if(send_raw_tof >= 0 && send_raw_tof < 4)
					{
						tcp_send_picture(100, 2, 160, 60, (uint8_t*)p_tof->raw_depth);
						tcp_send_picture(101, 2, 160, 60, (uint8_t*)p_tof->ampl_images[send_raw_tof]);
					}

					hmap_cnt = 0;

					if(send_pointcloud)
					{
						save_pointcloud(p_tof->n_points, p_tof->cloud);
					}
				}
			}

			static int32_t prev_x, prev_y, prev_ang;

			if(state_vect.v.mapping_3d && !pwr_status.charging && !pwr_status.charged)
			{
				if(p_tof->robot_pos.x != 0 || p_tof->robot_pos.y != 0 || p_tof->robot_pos.ang != 0)
				{
					int robot_moving = 0;
					if((prev_x != p_tof->robot_pos.x || prev_y != p_tof->robot_pos.y || prev_ang != p_tof->robot_pos.ang))
					{
						prev_x = p_tof->robot_pos.x; prev_y = p_tof->robot_pos.y; prev_ang = p_tof->robot_pos.ang;
						robot_moving = 1;
					}

					static int n_tofs_to_map = 0;
					static tof3d_scan_t* tofs_to_map[25];

					tofs_to_map[n_tofs_to_map] = p_tof;
					n_tofs_to_map++;

					if(n_tofs_to_map >= (robot_moving?3:20))
					{
						int32_t mid_x, mid_y;
						map_3dtof(&world, n_tofs_to_map, tofs_to_map, &mid_x, &mid_y);

						if(do_follow_route)
						{
							int px, py, ox, oy;
							page_coords(mid_x, mid_y, &px, &py, &ox, &oy);

							for(int ix=-1; ix<=1; ix++)
							{
								for(int iy=-1; iy<=1; iy++)
								{
									gen_routing_page(&world, px+ix, py+iy, 0);
								}
							}
						}

						n_tofs_to_map = 0;
					}
				}
			}

		}
#endif

		static double prev_sync = 0;
		double stamp;

		double write_interval = 30.0;
		if(tcp_client_sock >= 0)
			write_interval = 7.0;

		if( (stamp=subsec_timestamp()) > prev_sync+write_interval)
		{
			prev_sync = stamp;
			fflush(stdout); // syncs log file.
		}

	}

	request_tof_quit();

	return NULL;
}


void* start_tof(void*);

int main(int argc, char** argv)
{
	pthread_t thread_main, thread_tof, thread_tof2;

	int ret;

	if( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) )
	{
		printf("ERROR: main thread creation, ret = %d\n", ret);
		return -1;
	}

	if( (ret = pthread_create(&thread_tof, NULL, pulutof_poll_thread, NULL)) )
	{
		printf("ERROR: tof3d access thread creation, ret = %d\n", ret);
		return -1;
	}

	#ifndef PULUTOF1_GIVE_RAWS
		if( (ret = pthread_create(&thread_tof2, NULL, pulutof_processing_thread, NULL)) )
		{
			printf("ERROR: tof3d processing thread creation, ret = %d\n", ret);
			return -1;
		}
	#endif

	pthread_join(thread_main, NULL);

	pthread_join(thread_tof, NULL);
	#ifndef PULUTOF1_GIVE_RAWS
	pthread_join(thread_tof2, NULL);
	#endif

	return retval;
}

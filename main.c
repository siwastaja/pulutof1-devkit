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
#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
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

#include "tcp_comm.h"
#include "tcp_parser.h"

#include "pulutof.h"

volatile int verbose_mode = 0;
volatile int send_raw_tof = -1;
volatile int send_pointcloud = 0; // 0 = off, -1 = relative to origin to stdout, 1 = relative to robot to files, 2 = relative to actual world coords to files

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
	fprintf(stderr, "Saving pointcloud with %d samples to file %s.\n", n_points, fname);
	FILE* pc_csv = fopen(fname, "w");
	if(!pc_csv)
	{
	   fprintf(stderr, "Error opening file for write.\n");
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


void print_pointcloud(int n_points, xyz_t* cloud)
{
   for (int i = 0; i < n_points; i++) {
      printf("%d %d %d\n",cloud[i].x, -1*cloud[i].y, cloud[i].z);
   } // for
   
} // print_pointcloud


void request_tof_quit(void);

volatile int retval = 0;


void pulutof_set_exposure(int exposure_base)
{
   if (exposure_base > 10000) {
      fprintf(stderr, "ERROR: trying to set exposure base time too big (%d us), set to maximun (10000 us)\n", exposure_base);
      exposure_base = 10000;
   } else if (exposure_base < 10.0) {
      fprintf(stderr, "ERROR: trying to set exposure base time too small (%d us), set to minimun (10 us)\n", exposure_base);
   } // if-else

   pulutof_command(PULUTOF_COMMAND_EXPOSURE, exposure_base);
   
} // pulutof_set_exposure


void pulutof_set_hdr_multiplier(int hdr_multiplier)
{
   if (hdr_multiplier > 16) {
      fprintf(stderr, "ERROR: trying to set hdr-multiplier too big (%d), set to maximun (16)\n", hdr_multiplier); 
      hdr_multiplier = 16;
   } else if (hdr_multiplier < 2) {
      fprintf(stderr, "ERROR: trying to set hdr-multiplier too small (%d), set to minimun (2)\n", hdr_multiplier); 
      hdr_multiplier = 2;
   } // if-else

   pulutof_command(PULUTOF_COMMAND_HDR_MULTIPLIER, hdr_multiplier);

} // pulutof_set_exposure


void* main_thread()
{
   char buffer[80];

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
			fgets(buffer, sizeof(buffer), stdin);

			int cmd = buffer[0];
 		   
			if(cmd == 'q')
			{
				retval = 0;
				break;
			}
			if(cmd == 'z')
			{
				if(send_raw_tof >= 0) send_raw_tof--;
				fprintf(stderr, "INFO: Sending raw tof from sensor %d\n", send_raw_tof);
			}
			if(cmd == 'x')
			{
				if(send_raw_tof < 3) send_raw_tof++;
				fprintf(stderr, "INFO: Sending raw tof from sensor %d\n", send_raw_tof);
			}
			if(cmd >= '0' && cmd <= '3')
			{
			   fprintf(stderr, "Requesting offset calib\n");
			   pulutof_command(PULUTOF_COMMAND_CALIBRATE_OFFSET, cmd - '0');
			}
			if(cmd == 'v')
			{
				verbose_mode = verbose_mode?0:1;
			}
			if(cmd == 'p')
			{
				if (send_pointcloud == 0) {
				   fprintf(stderr, "INFO: Will send pointclouds relative to robot origin\n");
				   send_pointcloud = 1;
				} else if (send_pointcloud == 1) {
				   fprintf(stderr, "INFO: Will send pointclouds relative to world origin\n");
				   send_pointcloud = 2;
				} else {
				   fprintf(stderr, "INFO: Will stop sending pointclouds\n");
				   send_pointcloud = 0;
				} // if-else
			} // if
			if (cmd == 'm')
			{
			   sscanf(buffer+1, "%d", &cmd);
			   fprintf(stderr, "INFO: Set midlier remove filter %s\n", ((cmd==0)?"OFF":"ON"));
			   pulutof_command(PULUTOF_COMMAND_MIDLIER_FILTER, cmd != 0);
			} //if
			if (cmd == 'e')
			{
			   int exposure;
			   sscanf(buffer+1, "%d", &exposure);
			   fprintf(stderr, "INFO: Set exposure base time to %d microseconds\n", exposure);
			   pulutof_set_exposure(exposure);
			} //if
			if (cmd == 'h')
			{
			   int hdr_multiplier;
			   sscanf(buffer+1, "%d", &hdr_multiplier);
			   fprintf(stderr, "INFO: Set exposure time hdr-multiplier to %d\n", hdr_multiplier);
			   pulutof_set_hdr_multiplier(hdr_multiplier);
			} //if
		} // if




		if(tcp_client_sock >= 0 && FD_ISSET(tcp_client_sock, &fds))
		{
			int ret = handle_tcp_client();
			if(ret == TCP_CR_MAINTENANCE_MID)
			{
				if(msg_cr_maintenance.magic == 0x12345678)
				{
					retval = msg_cr_maintenance.retval;
					break;
				}
				else
				{
				        fprintf(stderr, "WARN: Illegal maintenance message magic number 0x%08x.\n", msg_cr_maintenance.magic);
				}
			}		
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
		}


		tof3d_scan_t *p_tof;
		
		if( (p_tof = get_tof3d()) )
		{
		   	if (send_pointcloud > 0) {
			   save_pointcloud(p_tof->n_points, p_tof->cloud);
			} else if (send_pointcloud < 0) {
			   print_pointcloud(p_tof->n_points, p_tof->cloud);
			} // if else

			if(tcp_client_sock >= 0)
			{
				if(send_raw_tof >= 0 && send_raw_tof < 4)
				{
					tcp_send_picture(100, 2, 160, 60, (uint8_t*)p_tof->raw_depth);
					tcp_send_picture(101, 2, 160, 60, (uint8_t*)p_tof->ampl_images[send_raw_tof]);
				}
			}

		}

	}

	request_tof_quit();

	return NULL;
}


void pulutof_print_info(char* command_name)
{
   fprintf(stderr,
	   "\n"
	   "Usage: %s [Options]\n"
	   "Options:\n"
	   " -p           \t A continuous pointcloud output to stdout\n"
	   " -m 0|1       \t Midlier filter off/on (default on)\n"
	   " -e 10..10000 \t Exposure time base in microseconds (default 80 us)\n"
	   " -h 2..16     \t Hdr-multiplier for exposure time (default 7)\n"
	   "\n"
	   "Exits with q\n\n",
	   command_name);
   
} // pulutof_print_info


int main(int argc, char** argv)
{
	pthread_t thread_main, thread_tof, thread_tof2;

	int ret, opt;
	
       
	if ( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) ) {	   
	   fprintf(stderr, "ERROR: main thread creation, ret = %d\n", ret);
	   return EXIT_FAILURE;
	} // if

	if ( (ret = pthread_create(&thread_tof, NULL, pulutof_poll_thread, NULL)) ) {
	   fprintf(stderr, "ERROR: tof3d access thread creation, ret = %d\n", ret);
	   return EXIT_FAILURE;
	} // if

	#ifndef PULUTOF1_GIVE_RAWS
	if ( (ret = pthread_create(&thread_tof2, NULL, pulutof_processing_thread, NULL)) ) {
	   fprintf(stderr, "ERROR: tof3d processing thread creation, ret = %d\n", ret);
	   return -1;
	} // if
	#endif

	usleep(10000); // gives processsor time for threads started above

	while ((opt = getopt(argc, argv, "pm:e:h:?")) != -1) {
	   switch (opt) {
	   case 'p':  
	      send_pointcloud = -1;
	      break;
	   case 'm':
	      pulutof_command(PULUTOF_COMMAND_MIDLIER_FILTER, *optarg != '0');
	      break;
	   case 'e':
	      pulutof_set_exposure(atoi(optarg));
	      break;
	   case 'h':
	      pulutof_set_hdr_multiplier(atoi(optarg));
	      break;
	   default: /* '?' */
	      pulutof_print_info(argv[0]);
	      exit(EXIT_FAILURE);
	   } // switch
	} // while

	if (argv[optind] != NULL) {
	   pulutof_print_info(argv[0]);
	   exit(EXIT_FAILURE);
	} // if

	pthread_join(thread_main, NULL);

	pthread_join(thread_tof, NULL);
	#ifndef PULUTOF1_GIVE_RAWS
	pthread_join(thread_tof2, NULL);
	#endif

	return retval;

} // main

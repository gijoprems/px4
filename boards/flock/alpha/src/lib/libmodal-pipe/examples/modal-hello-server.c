/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <modal_start_stop.h>
#include <modal_pipe_server.h>

#define SERVER_NAME	"modal-hello-server"
#define HELLO_PIPE_NAME	"hello"
#define HELLO_PIPE_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR HELLO_PIPE_NAME "/")
#define CH			0 // arbitrarily use channel 0 for our one pipe

// vars
static int en_debug;
static double frequency_hz = 2.0;


// printed if some invalid argument was given
static void print_usage(void)
{
	printf("\n\
modal-hello-server usually runs as a systemd background service. However, for debug\n\
purposes it can be started from the command line manually with any of the following\n\
debug options. When started from the command line, modal-hello-server will automatically\n\
stop the background service so you don't have to stop it manually\n\
\n\
modal-hello-server also creates a control pipe to test sending commands back to\n\
the server from either a client or from the command line. To test, try this:\n\
echo -n test > /run/mpa/hello/control\n\
\n\
-d, --debug                 print debug info\n\
-f, --frequency             publish frequency in hz\n\
-h, --help                  print this help message\n\
\n");
	return;
}


static void control_pipe_handler(int ch, char* string, int bytes, __attribute__((unused)) void* context)
{
	printf("received command on channel %d bytes: %d string: \"%s\"\n", ch, bytes, string);
	return;
}


static void connect_handler(int ch, int client_id, char* client_name, __attribute__((unused)) void* context)
{
	printf("client \"%s\" connected to channel %d  with client id %d\n", client_name, ch, client_id);
	return;
}


static void disconnect_handler(int ch, int client_id, char* name, __attribute__((unused)) void* context)
{
	printf("client \"%s\" with id %d has disconnected from channel %d\n", name, client_id, ch);
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"debug",			no_argument,		0,	'd'},
		{"frequency",		required_argument,	0,	'f'},
		{"help",			no_argument,		0,	'h'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "df:h", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'd':
			en_debug = 1;
			break;

		case 'f':
			frequency_hz = atof(optarg);
			if(frequency_hz<0.5){
				fprintf(stderr, "ERROR: frequency must be > 0.5hz\n");
				return -1;
			}
			break;

		case 'h':
			print_usage();
			return -1;

		default:
			print_usage();
			return -1;
		}
	}

	return 0;
}



int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

////////////////////////////////////////////////////////////////////////////////
// gracefully handle an existing instance of the process and associated PID file
////////////////////////////////////////////////////////////////////////////////

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(kill_existing_process(SERVER_NAME, 2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

////////////////////////////////////////////////////////////////////////////////
// set up the pipe
////////////////////////////////////////////////////////////////////////////////

	// enable the control pipe feature and optionally debug prints
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;
	if(en_debug) flags|=SERVER_FLAG_EN_DEBUG_PRINTS;

	// configure optional callbacks
	pipe_server_set_control_cb(CH, &control_pipe_handler, NULL);
	pipe_server_set_connect_cb(CH, &connect_handler, NULL);
	pipe_server_set_disconnect_cb(CH, &disconnect_handler, NULL);

	// create the pipe
	pipe_info_t info = { \
		.name        = HELLO_PIPE_NAME,\
		.location    = HELLO_PIPE_LOCATION ,\
		.type        = "text",\
		.server_name = SERVER_NAME,\
		.size_bytes  = MODAL_PIPE_DEFAULT_PIPE_SIZE};

	if(pipe_server_create(CH, info, flags)) return -1;

	// add in an optional field to the info JSON file
	cJSON* info_json = pipe_server_get_info_json_ptr(CH);
	cJSON_AddStringToObject(info_json, "description", "Test pipe sends hello text messages");
	pipe_server_update_info(CH);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(SERVER_NAME);

////////////////////////////////////////////////////////////////////////////////
// all threads started, wait for signal handler to stop it
////////////////////////////////////////////////////////////////////////////////

	main_running=1; // this is an extern variable in start_stop.c
	char str[32];
	char len;
	int i = 0;

	printf("Init complete, entering main loop\n");
	while(main_running){

		// make new string to send
		len = sprintf(str,"hello%d", i);
		i++;

		// send to first channel
		if(en_debug){
			int n = pipe_server_get_num_clients(0);
			printf("sending \"%s\" to %d connected clients\n", str, n);
		}
		pipe_server_write(0, str, len+1);

		// rough publish rate
		usleep(1000000/frequency_hz);
	}

////////////////////////////////////////////////////////////////////////////////
// Stop all the threads and do cleanup HERE
////////////////////////////////////////////////////////////////////////////////

	printf("Starting shutdown sequence\n");
	pipe_server_close_all();
	remove_pid_file(SERVER_NAME);
	printf("exiting cleanly\n");
	return 0;
}

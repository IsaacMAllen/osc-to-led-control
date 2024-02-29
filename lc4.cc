#include <iostream>

#ifndef WIN32
#include <unistd.h>
#endif

#include <sys/soundcard.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <lo/lo.h>
#include <lo/lo_cpp.h>
#include <cmath>

#include "led-matrix.h"

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define MIDI_DEVICE "/dev/sequencer"

int done = 0;
int count = 0;
int red = 0;
int green = 0;
int blue = 0;

void error(int num, const char *m, const char *path);

int echo_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);

int quit_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);

int brightness_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);

int red_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);
int green_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);
int blue_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
	interrupt_received = true;
}

static void DrawOnCanvas(RGBMatrix *canvas, uint8_t note, uint8_t velocity) {
	uint8_t brightness = (float) velocity / 127.0 * 255;
	canvas->SetBrightness(brightness);
	switch (note % 3) {
		case 0:
			canvas->Fill(0, 0, 255);
			break;
		case 1:
			canvas->Fill(0, 255, 0);
			break;
		case 2:
			canvas->Fill(255, 0, 0);
			break;
	}	
}

int main(int argc, char *argv[]) {

	const char *port = "9000";

	lo_server_thread st = lo_server_thread_new_with_proto(port, LO_UDP, error);
	if (!st) {
		std::cout << "Unable to start server thread\n";
		return 1;
	}


	RGBMatrix::Options defaults;
	defaults.hardware_mapping = "adafruit-hat";
	defaults.rows = 32;
	defaults.chain_length = 2;
	defaults.parallel = 1;
	defaults.show_refresh_rate = false;
	RGBMatrix *canvas = RGBMatrix::CreateFromFlags(&argc, &argv, &defaults);
	if (canvas == NULL)
		return 1;

	signal(SIGTERM, InterruptHandler);
	signal(SIGINT, InterruptHandler);

	lo_server_thread_add_method(st, "/quit", "", quit_handler, NULL);

	lo_server_thread_add_method(st, "/lc4/brightness", NULL, brightness_handler, canvas);
	
	lo_server_thread_add_method(st, "/lc4/red", NULL, red_handler, canvas);
	lo_server_thread_add_method(st, "/lc4/green", NULL, green_handler, canvas);
	lo_server_thread_add_method(st, "/lc4/blue", NULL, blue_handler, canvas);

	lo_server_thread_start(st);

	std::cout << "listening on udp port " << port << std::endl;


	unsigned char inpacket[4];
	// first open the sequencer device for reading
	int seqfd = open(MIDI_DEVICE, O_RDONLY);
	if (seqfd == -1) {
		printf("Error: cannot open %s\n", MIDI_DEVICE);
		printf("errno: %d\n", errno);
		exit(1);
	}
	while(!interrupt_received) {


//		read(seqfd, &inpacket, sizeof(inpacket));
//		if (inpacket[0] == MIDI_NOTEON) {
//			DrawOnCanvas(canvas, inpacket[1], inpacket[2]);
//			//printf("Midi note: %d Velocity: %d\n", inpacket[1], inpacket[2]);
//			usleep(1*1000);
//		}
	}

	canvas->Clear();
	delete canvas;
	lo_server_thread_free(st);
	return 0;
}


void error(int num, const char *msg, const char *path)
{
	printf("liblo server error %d in path %s: %s\n", num, path, msg);
	fflush(stdout);
}

int brightness_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *canvas) {
	int amount = argv[0]->i;
	((RGBMatrix *)canvas)->SetBrightness(amount);
	return 0;
}


int red_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *canvas) {
	red = argv[0]->i;
	((RGBMatrix *)canvas)->Fill(red, green, blue);
	return 0;
}


int green_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *canvas) {
	green = argv[0]->i;
	((RGBMatrix *)canvas)->Fill(red, green, blue);
	return 0;
}


int blue_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *canvas) {
	blue = argv[0]->i;
	((RGBMatrix *)canvas)->Fill(red, green, blue);
	return 0;
}

/* catch any incoming messages, display them, and send them
 * back. */
int echo_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data)
{
	int i;
	lo_message m = (lo_message)data;
	lo_address a = lo_message_get_source(m);
	lo_server s = (lo_server)user_data;
	const char *host = lo_address_get_hostname(a);
	const char *port = lo_address_get_port(a);

	count++;

#ifdef WIN32
	Sleep(1);
#else
	sleep(1);
#endif

	printf("path: <%s>\n", path);
	for (i = 0; i < argc; i++) {
		printf("arg %d '%c' ", i, types[i]);
		lo_arg_pp((lo_type)types[i], argv[i]);
		printf("\n");
	}

	if (!a) {
		printf("Couldn't get message source, quitting.\n");
		done = 1;
		return 0;
	}

	int r = lo_send_message_from(a, s, path, m);
	if (r < 0)
		printf("Error sending back message, socket may have closed.\n");
	else
		printf("Sent message back to %s:%s.\n", host, port);

	if (count >= 3) {
		printf("Got enough messages, quitting.\n");
		done = 1;
	}

	return 0;
}

int quit_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data)
{
	done = 1;
	printf("quitting\n\n");
	fflush(stdout);

	return 0;
}


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

#include "led-matrix.h"

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define MIDI_DEVICE "/dev/sequencer"

int done = 0;
int count = 0;

void error(int num, const char *m, const char *path);

int echo_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);

int quit_handler(const char *path, const char *types, lo_arg ** argv,
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

	/*
	 * Create a server on a background thread.  Note, all message
	 * handlers will run on the background thread!
	 */
	//	lo::ServerThread st(9000);
	//	if (!st.is_valid()) {
	//		std::cout << "Nope." << std::endl;
	//		return 1;
	//	}

	/* Set some lambdas to be called when the thread starts and
	 * ends. Here we demonstrate capturing a reference to the server
	 * thread. */
	//	st.set_callbacks([&st](){printf("Thread init: %p.\n",&st);},
	//			[](){printf("Thread cleanup.\n");});
	//
	//	std::cout << "URL: " << st.url() << std::endl;
	//
	//	st.add_method("VOLUME_L", "i",
	//			[](lo_arg **argv, int)
	//			{std::cout << "volume_L (" << std::endl;});
	//
	//	st.start();
	//	
	//	lo::Address a("rasberrypi", "9000");
	//	a.send("VOLUME_L", "i", 1);

	const char *port = "9000";
	const char *group = "255.0.0.1";

	lo_server_thread st = lo_server_thread_new_with_proto(port, LO_UDP, error);
	if (!st) {
		std::cout << "Unable to start server thread\n";
		return 1;
	}

	lo_server s = lo_server_thread_get_server(st);

	lo_server_thread_add_method(st, "/quit", "", quit_handler, NULL);

	lo_server_thread_add_method(st, NULL, NULL, echo_handler, s);

	lo_server_thread_start(st);

	std::cout << "listening on udp port " << port << std::endl;


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

	unsigned char inpacket[4];
	// first open the sequencer device for reading
	int seqfd = open(MIDI_DEVICE, O_RDONLY);
	if (seqfd == -1) {
		printf("Error: cannot open %s\n", MIDI_DEVICE);
		printf("errno: %d\n", errno);
		exit(1);
	}
	while(!interrupt_received) {

		// now just wait around for MIDI bytes to arrive and print them to screen.

		read(seqfd, &inpacket, sizeof(inpacket));
		if (inpacket[0] == MIDI_NOTEON) {
			DrawOnCanvas(canvas, inpacket[1], inpacket[2]);
			//printf("Midi note: %d Velocity: %d\n", inpacket[1], inpacket[2]);
			usleep(1*1000);
		}
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


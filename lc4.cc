#include <sys/soundcard.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "led-matrix.h"

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define MIDI_DEVICE "/dev/sequencer"

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
	
	return 0;
}


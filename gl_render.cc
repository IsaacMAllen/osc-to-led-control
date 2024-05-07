#include <iostream>

#ifndef WIN32
#include <unistd.h>
#endif

#ifndef TARGET_UID
#define TARGET_UID 0
#endif

#ifndef TARGET_GID
#define TARGET_GID 0
#endif

#ifndef UID_MIN
#define UID_MIN 500
#endif

#ifndef GID_MIN
#define GID_MIN 500
#endif

#include <vector>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/soundcard.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <lo/lo.h>
#include <lo/lo_cpp.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <fcntl.h>
#include <unistd.h>
#include "led-matrix.h"
#include <time.h>
#include "shadergl.h"
#include <raylib.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include <time.h>
#include <sys/time.h>
#include <sys/inotify.h>
#include <fcntl.h>
#include <limits.h>
#include <complex.h>
#include <fftw3.h>
#include <math.h>
#include <cmath>
#include <portaudio.h>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;
using namespace std::complex_literals;

#define MIDI_DEVICE "/dev/sequencer"
#define BUF_LEN (10 * (sizeof(struct inotify_event) + NAME_MAX + 1))
#define FPS 160
#define SAMPLE_RATE 44100.0
#define FRAMES_PER_BUFFER 512
#define NUM_CHANNELS 1
#define SPECTRO_FREQ_START 20
#define SPECTRO_FREQ_END 20000

typedef struct {
   double *in;
   double *out;
   fftw_plan p;
   int startIndex;
   int spectroSize;
} streamCallbackData;

static streamCallbackData* spectroData;
//fftw_complex *in, *out;
PaStream *stream;
//float *in;
//std::complex<float> *out;
fftw_plan fftwPlan;
typedef struct {
   float left;
   float right;
} Frame;
double max_amp;
Frame global_frames[4080] = {0};
size_t global_frames_count = 0;
int done = 0;
int count = 0;
int serial_port;
int brightness = 0;
float red = 0.0;
float green = 0.0;
float blue = 0.0;
float white = 0.0;
int panelWidth = 64;
int panelHeight = 64;
float exponent = 0.0;
float x = 0.5;
float y = 0.5;
int shader = 1;
int device;
bool bound = false;
unsigned int audioTexture;
drmModeModeInfo mode;
struct gbm_device *gbmDevice;
struct gbm_surface *gbmSurface;
drmModeCrtc *crtc;
RGBMatrix * canvas;
uint32_t connectorId;
float *test;
GLuint vao;
int width, height, nrChannels;
unsigned char *data;

void error(int num, const char *m, const char *path);

int echo_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);

int quit_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);

int brightness_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);

int red_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *colorLoc);
int green_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *colorLoc);
int blue_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *colorLoc);
int white_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *colorLoc);
int clear_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *colorLoc);
int exponent_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);
int x_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);
int y_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);
int shader_handler(const char *path, const char *types, lo_arg ** argv,
      int argc, lo_message data, void *user_data);

void checkErrGL(int line);

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
   interrupt_received = true;
}

//   for (size_t f = 0; f < n; ++f) {
//      out[f] = 0;
//      for (size_t i = 0; i < n; i++) {
//         double t = (double)i/n;
//         out[f] += in[i]*std::exp(std::complex<double>(2 * pi * f * t) * 1i);
//      }
//      out[f] = 0;
//      for (size_t i = 0; i < n; ++i) {
//         double t = (double)i/n;
//         out[f] += in[i]*std::exp(std::complex<double>(2 * pi * f * t) * 1i);
//      } 
//   }


#define ARRAY_LEN(xs) sizeof(xs)/sizeof(xs[0])

void fft(float in[], size_t stride, std::complex<float> out[], size_t n) {
   assert(n > 0);

   if (n == 1) {
      out[0] = in[0];
      return;
   }

   fft(in, stride*2, out, n/2);
   fft(in + stride, stride*2, out + n/2, n/2);

   for (size_t k = 0; k < n/2; ++k) {
      float t = (float)k/n;
      std::complex<float> v = std::exp(std::complex<float>(-2*M_PI*t) * std::complex<float>(1i)) * out[k + n/2];
      std::complex<float> e = out[k];
      out[k] = e + v;
      out[k + n/2] = e - v;
   }
}


//void audioCallback(void *bufferData, unsigned int frames) {
//   //   size_t capacity = ARRAY_LEN(global_frames);
//   //   if (frames <= capacity - global_frames_count) {
//   //      memcpy(global_frames + global_frames_count, bufferData, sizeof(Frame)*frames);
//   //      global_frames_count += frames;
//   //   } else if(frames <= capacity) {
//   //      memmove(global_frames, global_frames + frames, sizeof(Frame)*(capacity - frames));
//   //      memcpy(global_frames + (capacity - frames), bufferData, sizeof(Frame)*frames);
//   //   } else {
//   //      memcpy(global_frames, bufferData, sizeof(Frame)*capacity);
//   //      global_frames_count = capacity;
//   //   }
//   //if (frames < N) return;
//   Frame *fs = (Frame *)bufferData;
//   for (size_t i = 0; i < frames; ++i) {
//      //in[i][0] = fs[i].left;
//      in[i] = fs[i].left;
//      //in[i][1] = 0.0;
//   }
//   //fftw_execute(fftwPlan);
//   fft(in, 1, out, N); 
//}

static drmModeConnector *getConnector(drmModeRes *resources) {
   for (int i = 0; i < resources -> count_connectors; i++) {
      drmModeConnector *connector = drmModeGetConnector(device, resources->connectors[i]);
      if (connector->connection == DRM_MODE_CONNECTED) {
         return connector;
      }
      drmModeFreeConnector(connector);
   }

   return NULL;
}

static drmModeEncoder *findEncoder(drmModeConnector *connector) {
   if (connector->encoder_id) {
      return drmModeGetEncoder(device, connector->encoder_id);   
   }
   return NULL;
}

static int getDisplay(EGLDisplay *display) {
   drmModeRes *resources = drmModeGetResources(device);
   if (resources == NULL) {
      fprintf(stderr, "Unable to get DRM resources\n");
      std::cout << errno << std::endl;
      return -1;
   }
   drmModeConnector *connector = getConnector(resources);
   if (connector == NULL) {
      fprintf(stderr, "Unable to get connector\n");
      drmModeFreeResources(resources);
      return -1;
   }

   connectorId = connector->connector_id;
   mode = connector->modes[0];

   drmModeEncoder *encoder = findEncoder(connector);
   if (encoder == NULL) {
      fprintf(stderr, "Unable to get encoder\n");
      drmModeFreeConnector(connector);
      drmModeFreeResources(resources);
      return -1;
   }

   crtc = drmModeGetCrtc(device, encoder->crtc_id);
   drmModeFreeEncoder(encoder);
   drmModeFreeConnector(connector);
   drmModeFreeResources(resources);
   gbmDevice = gbm_create_device(device);
   gbmSurface = gbm_surface_create(gbmDevice, mode.hdisplay, mode.vdisplay, GBM_FORMAT_XRGB8888, GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
   *display = eglGetDisplay(gbmDevice);
   return 0;
}

static int matchConfigToVisual(EGLDisplay display, EGLint visualId, EGLConfig *configs, int count) {
   EGLint id;
   for (int i = 0; i < count; ++i) {
      if (!eglGetConfigAttrib(display, configs[i], EGL_NATIVE_VISUAL_ID, &id))
         continue;
      if (id == visualId)
         return i;
   }
   return -1;
}

static struct gbm_bo *previousBo = NULL;
static uint32_t previousFb;

static void gbmSwapBuffers(EGLDisplay *display, EGLSurface *surface) {
   eglSwapBuffers(*display, *surface);
   struct gbm_bo *bo = gbm_surface_lock_front_buffer(gbmSurface);
   uint32_t handle = gbm_bo_get_handle(bo).u32;
   uint32_t pitch = gbm_bo_get_stride(bo);
   uint32_t fb;
   drmModeAddFB(device, mode.hdisplay, mode.vdisplay, 24, 32, pitch, handle, &fb);
   drmModeSetCrtc(device, crtc->crtc_id, fb, 0, 0, &connectorId, 1, &mode);

   if (previousBo) {
      drmModeRmFB(device, previousFb);
      gbm_surface_release_buffer(gbmSurface, previousBo);
   }
   previousBo = bo;
   previousFb = fb;
}

static void gbmClean() {
   //set the previous crtc
   drmModeSetCrtc(device, crtc->crtc_id, crtc->buffer_id, crtc->x, crtc->y, &connectorId, 1, &crtc->mode);
   drmModeFreeCrtc(crtc);

   if (previousBo) {
      drmModeRmFB(device, previousFb);
      gbm_surface_release_buffer(gbmSurface, previousBo);
   }

   gbm_surface_destroy(gbmSurface);
   gbm_device_destroy(gbmDevice);
}

// The following code was adopted from
// https://github.com/matusnovak/rpi-opengl-without-x/blob/master/triangle.c
// and is licensed under the Unlicense
static const EGLint configAttribs[] = {
   EGL_RED_SIZE, 8,
   EGL_GREEN_SIZE, 8,
   EGL_BLUE_SIZE, 8,
   EGL_DEPTH_SIZE, 8,
   EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
   EGL_NONE
};

static const EGLint contextAttribs[] = {
   EGL_CONTEXT_MAJOR_VERSION, 3,
   EGL_CONTEXT_MINOR_VERSION, 0,
   //EGL_CONTEXT_CLIENT_VERSION, 2,
   EGL_NONE
};

// Get the EGL error back as a string. Useful for debugging.
static const char *eglGetErrorStr() {
   switch(eglGetError()) {
      case EGL_SUCCESS:
         return "The last function succeeded without error.";
      case EGL_NOT_INITIALIZED:
         return "EGL is not initialized, or could not be initialized, for the specified EGL display connection.";
      case EGL_BAD_ACCESS:
         return "EGL cannot access a requested resource (for example a context is bound in another thread).";
      case EGL_BAD_ALLOC:
         return "EGL failed to allocate resources for the requested operation.";
      case EGL_BAD_ATTRIBUTE:
         return "An unrecognized attribute or attibute value was passed in the attribute list.";
      case EGL_BAD_CONTEXT:
         return "An EGLContext argument does not name a valid EGL rendering context.";
      case EGL_BAD_CONFIG:
         return "An EGLConfig argument does not name a valid EGL frame buffer configuration.";
      case EGL_BAD_CURRENT_SURFACE:
         return "The current surface of the calling thread is a window, pixel "
            "buffer or pixmap that is no longer valid.";
      case EGL_BAD_DISPLAY:
         return "An EGLDisplay argument does not name a valid EGL display connection.";
      case EGL_BAD_SURFACE:
         return "An EGLSurface argument does not name a valid surface (window, pixel buffer, or pixmap) "
            "configured for GL rendering.";
      case EGL_BAD_MATCH:
         return "Arguments are inconsistent (for example, a valid context requires buffers not supplied "
            "by a valid surface).";
      case EGL_BAD_PARAMETER:
         return "One or more argument values are invalid.";
      case EGL_BAD_NATIVE_PIXMAP:
         return "A NativePixmapType argument does not refer to a valid native pixmap.";
      case EGL_BAD_NATIVE_WINDOW:
         return "A NativeWindowType argument does not refer to a valid native window.";
      case EGL_CONTEXT_LOST:
         return "A power management event has occurred. The application must destroy all "
            "contexts and reinitialize OpenGL ES state and objects to continue rendering.";
      default:
         break;
   }
   return "Unknown error!";
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

static void checkPAErr(PaError err) {
   if (err != paNoError) {
      printf("PortAudio error: %s\n", Pa_GetErrorText(err));
      exit(EXIT_FAILURE);
   }
}

static inline float max(float a, float b) {
   return a > b ? a : b;
}

static inline float min(float a, float b) {
   return a < b ? a : b;
}

static int audioStreamCallback(
      const void *inputBuffer,
      void *outputBuffer,
      unsigned long framesPerBuffer,
      const PaStreamCallbackTimeInfo *timeInfo,
      PaStreamCallbackFlags statusFlags,
      void *userData 
      ) {
   
   float *in = (float *)inputBuffer;
   (void)outputBuffer;
   streamCallbackData *callbackData = (streamCallbackData *)userData;   
   
   for (unsigned long i = 0; i < framesPerBuffer; i++) {
      callbackData->in[i] = in[i * NUM_CHANNELS];
   }
   
   fftw_execute(callbackData->p);
   // Draw the spectrogram
   int dispSize = 64;
   for (int i = 0; i < 64; i++) {
         for (int j = 0; j < 64; j++) {
            test[i*64 + j] = 1.0;
         }
   }
    for (int j = 0; j < dispSize; j++) {
        // Sample frequency data logarithmically
        double proportion = std::pow(j / (double)dispSize, 2);
        double freq = callbackData->out[(int)(callbackData->startIndex
            + proportion * callbackData->spectroSize)];

        // Display full block characters with heights based on frequency intensity
      //   double height = 0.0;
      //   if (freq < 0.05) {
      //       height = 0.0;
      //   } else if (freq < 0.125) {
      //       height = 1.0 / 8.0;
      //   } else if (freq < 0.25) {
      //       height = 2.0 / 8.0;
      //   } else if (freq < 0.375) {
      //       height = 3.0 / 8.0;
      //   } else if (freq < 0.5) {
      //       height = 0.5;
      //   } else if (freq < 0.625) {
      //       height = 5.0 / 8.0;
      //   } else if (freq < 0.75) {
      //       height = 6.0 / 8.0;
      //   } else if (freq < 0.875) {
      //       height = 7.0 / 8.0;
      //   } else {
      //       height = 1.0;
      //   }
      //printf("col: %d, height: %f\r", j, height);
      //fflush(stdout);
      double height = 64.0 * freq;
      if (height > 64.0) height = 64.0;
      for (int i = 0; i < height; i++) {
         test[i*64 + j] = 0.5;
      }
   }
   
    // Display the buffered changes to stdout in the terminal
   
   //  glBindVertexArray(vao);
   // // Upload the audio data to the texture
   //  glBindTexture(GL_TEXTURE_2D, audioTexture);

    
   //   // set the texture wrapping parameters
   //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
   //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   //  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   // float * test = (float *)malloc(64*64*sizeof(float));
   // for (int i = 0; i < 4096; i++) {
   //    test[i] = 1.0;
   // }
   // glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 64, 64, 0, GL_RED, GL_FLOAT, test);
   // glGenerateMipmap(GL_TEXTURE_2D);
   // bound = true;
   // free(test);
   return 0; 
}

void checkErrGL(int line) {
   int err = glGetError();
   if (err != 0) {
      std::cout << line << ": " << err << std::endl;
   }
}

int portAudioInit() {

   PaError err;
   err = Pa_Initialize();
   checkPAErr(err);
   
   spectroData = (streamCallbackData *)malloc(sizeof(streamCallbackData));
   spectroData->in = (double *)malloc(sizeof(double) * FRAMES_PER_BUFFER);
   spectroData->out = (double *)malloc(sizeof(double) * FRAMES_PER_BUFFER);
   if (spectroData->in == NULL || spectroData->out == NULL) {
      printf("Could not allocate spectro data\n");
      exit(EXIT_FAILURE);
   }
   spectroData->p = fftw_plan_r2r_1d(
      FRAMES_PER_BUFFER, spectroData->in, spectroData->out, FFTW_R2HC, FFTW_MEASURE
   );
   double sampleRatio = FRAMES_PER_BUFFER / SAMPLE_RATE;
   spectroData->startIndex = std::ceil(sampleRatio * SPECTRO_FREQ_START); 
   spectroData->spectroSize = min(std::ceil(sampleRatio * SPECTRO_FREQ_END),
                                  FRAMES_PER_BUFFER / 2.0) - spectroData->startIndex;
   
   int numDevices = Pa_GetDeviceCount();
   printf("Number of PA devices: %d\n", numDevices);
   if (numDevices < 0) {
      printf("Error getting device count.\n");
      exit(EXIT_FAILURE);
   } else if (numDevices == 0) {
      printf("There are no available audio devices on this machine.\n");
      exit(EXIT_SUCCESS);
   }

   const PaDeviceInfo *deviceInfo;
   for (int i = 0; i < numDevices; i++) {
      deviceInfo = Pa_GetDeviceInfo(i);
      printf("Device %d:\n", i);
      printf(" name: %s\n", deviceInfo->name);
      printf(" maxInputChannels: %d\n", deviceInfo->maxInputChannels);
      printf(" maxOutputChannels: %d\n", deviceInfo->maxOutputChannels);
      printf(" defaultSampleRate: %f\n", deviceInfo->defaultSampleRate);
   }

   int device = 1;

   PaStreamParameters inputParameters;
   PaStreamParameters outputParameters;

   memset(&inputParameters, 0, sizeof(inputParameters));
   inputParameters.channelCount = NUM_CHANNELS;
   inputParameters.device = device;
   inputParameters.hostApiSpecificStreamInfo = NULL;
   inputParameters.sampleFormat = paFloat32;
   inputParameters.suggestedLatency = Pa_GetDeviceInfo(device)->defaultLowInputLatency;

   err = Pa_OpenStream(
         &stream,
         &inputParameters,
         NULL,
         SAMPLE_RATE,
         FRAMES_PER_BUFFER,
         paNoFlag,
         audioStreamCallback,
         spectroData
         );
   checkPAErr(err);

   return 0;
}

void updateTexture() {
   glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 64, 64, 0, GL_RED, GL_FLOAT, test);
   //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
   glGenerateMipmap(GL_TEXTURE_2D);
}

int main(int argc, char *argv[]) {

   // // Initialize OpenGL and create the audio texture
   //  glGenTextures(1, &audioTexture);
   //  checkErrGL(563);


   //Pa_Sleep(10 * 1000);   
   
   //double in[N];
   //std::complex<double> out[N];
   //in = (float *) malloc(sizeof(float) * N);
   //out = (std::complex<float> *) malloc (sizeof(std::complex<float>) * N);
   //in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
   //out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
   //fftwPlan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_MEASURE);
   //fft(in, 1, out, N);

   serial_port = open("/dev/ttyACM0", O_RDWR);
   if (serial_port < 0) {
      serial_port = open("/dev/ttyACM1", O_RDWR);
      if (serial_port < 0) {
         printf("Error %i from open: %s\n", errno, strerror(errno));
      }
   }
   struct termios tty;
   if (tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
   }
   //InitAudioDevice();
   portAudioInit();
   //Music music = LoadMusicStream("ecstasy.mp3");
   //assert(music.stream.sampleSize == 32);
   //assert(music.stream.channels == 2); 
   //printf("music.frameCount = %u\n", music.frameCount);
   //printf("music.stream.sampleRate = %u\n", music.stream.sampleRate);
   //printf("music.stream.sampleSize = %u\n", music.stream.sampleSize);
   //printf("music.stream.channels = %u\n", music.stream.channels);

   tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
   tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
   tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
   tty.c_cflag |= CS8; // 8 bits per byte (most common)
   tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
   tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
   tty.c_lflag &= ~ICANON;
   tty.c_lflag &= ~ECHO; // Disable echo
   tty.c_lflag &= ~ECHOE; // Disable erasure
   tty.c_lflag &= ~ECHONL; // Disable new-line echo
   tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
   tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
   tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
   tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed   
   tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
   tty.c_cc[VMIN] = 0;

   // Set in/out baud rate to be 115200
   cfsetispeed(&tty, B115200);
   cfsetospeed(&tty, B115200);

   // Save tty settings, also checking for error
   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
   }
   const char *port = "9000";

   lo_server_thread st = lo_server_thread_new_with_proto(port, LO_UDP, error);
   if (!st) {
      std::cout << "Unable to start server thread\n";
      return 1;
   }

   uid_t ruid, euid, suid;
   gid_t rgid, egid, sgid;
   int uerr, gerr, fd;

   if (getresuid(&ruid, &euid, &suid) == -1) {
      fprintf(stderr, "Cannot obtain user identity: %m.\n");
      return EXIT_FAILURE;
   }
   if (getresgid(&rgid, &egid, &sgid) == -1) {
      fprintf(stderr, "Cannot obtain group identity: %m.\n");
      return EXIT_FAILURE;
   }
   if (ruid != (uid_t)TARGET_UID && ruid < (uid_t)UID_MIN) {
      fprintf(stderr, "Invalid user.\n");
      return EXIT_FAILURE;
   }
   if ((rgid != (gid_t)TARGET_UID) && rgid < (gid_t)GID_MIN) {
      fprintf(stderr, "Invalid group.\n");
      return EXIT_FAILURE;
   }

   if (seteuid((uid_t)TARGET_UID) == -1) {
      fprintf(stderr, "Insufficient user privileges.\n");
      return EXIT_FAILURE;
   }

   if (setegid((gid_t)TARGET_GID) == -1) {
      fprintf(stderr, "Insufficient group privileges.\n");
      return EXIT_FAILURE;
   }

   RGBMatrix::Options defaults;
   defaults.hardware_mapping = "adafruit-hat";
   defaults.rows = 32;
   defaults.chain_length = 2;
   defaults.parallel = 1;
   defaults.show_refresh_rate = false;
   canvas = RGBMatrix::CreateFromFlags(&argc, &argv, &defaults);
   if (canvas == NULL)
      return 1;

   //   gerr = 0;
   //   if (setresgid(rgid, rgid, rgid) == -1) {
   //      gerr = errno;
   //      if (!gerr)
   //         gerr = EINVAL;
   //   }
   //   uerr = 0;
   //   if (setresuid(ruid, ruid, ruid) == -1) {
   //      uerr = errno;
   //      if (!uerr)
   //         uerr = EINVAL;
   //   }
   //   if (uerr || gerr) {
   //      if (uerr)
   //         fprintf(stderr, "Cannot drop user privileges: %s.\n", strerror(uerr));
   //      if (gerr)
   //         fprintf(stderr, "Cannot drop group privileges: %s.\n", strerror(gerr));
   //      return EXIT_FAILURE;
   //   }

   signal(SIGTERM, InterruptHandler);
   signal(SIGINT, InterruptHandler);

   EGLDisplay display;

   device = open("/dev/dri/card1", O_RDWR | O_CLOEXEC);
   if (getDisplay(&display) != 0) {
      device = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
      if (getDisplay(&display) != 0) {
         fprintf(stderr, "Unable to get EGL display\n");
         close(device);
         return -1;
      }
   }


   int major, minor;
   GLuint program, vert, frag, vbo, ebo;
   GLint posLoc, texLoc, result;

   if (eglInitialize(display, &major, &minor) == EGL_FALSE) {
      fprintf(stderr, "Failed to get EGL version! Error: %s\n",
            eglGetErrorStr());
      eglTerminate(display);
      gbmClean();
      return EXIT_FAILURE;
   }

   // Make sure that we can use OpenGL in this EGL app.
   eglBindAPI(EGL_OPENGL_API);

   printf("Initialized EGL version: %d.%d\n", major, minor);

   EGLint count;
   EGLint numConfigs;
   eglGetConfigs(display, NULL, 0, &count);
   EGLConfig *configs = (EGLConfig *) malloc(count * sizeof(configs));

   if (!eglChooseConfig(display, configAttribs, configs, count, &numConfigs)) {
      fprintf(stderr, "Failed to get EGL configs! Error: %s\n", eglGetErrorStr());
      eglTerminate(display);
      gbmClean();
      return EXIT_FAILURE;
   }

   // not sure why the EGL config must match the GBM format but it works!
   int configIndex = matchConfigToVisual(display, GBM_FORMAT_XRGB8888, configs, numConfigs);
   if (configIndex < 0) {
      fprintf(stderr, "Failed to find matching EGL config! Error: %s\n",
            eglGetErrorStr());
      eglTerminate(display);
      gbm_surface_destroy(gbmSurface);
      gbm_device_destroy(gbmDevice);
      return EXIT_FAILURE;
   }

   EGLContext context =
      eglCreateContext(display, configs[configIndex], EGL_NO_CONTEXT, contextAttribs);
   if (context == EGL_NO_CONTEXT) {
      fprintf(stderr, "Failed to create EGL context! Error: %s\n",
            eglGetErrorStr());
      eglTerminate(display);
      gbmClean();
      return EXIT_FAILURE;
   }

   EGLSurface surface =
      eglCreateWindowSurface(display, configs[configIndex], gbmSurface, NULL);
   if (surface == EGL_NO_SURFACE) {
      fprintf(stderr, "Failed to create EGL surface! Error: %s\n",
            eglGetErrorStr());
      eglDestroyContext(display, context);
      eglTerminate(display);
      gbmClean();
      return EXIT_FAILURE;
   }

   free(configs);
   eglMakeCurrent(display, surface, surface, context);

   // Set GL Viewport size, always needed!
   glViewport(0, 0, panelWidth, panelHeight);

   // Get GL Viewport size and test if it is correct.
   GLint viewport[4];
   glGetIntegerv(GL_VIEWPORT, viewport);

   if (viewport[2] != panelWidth || viewport[3] != panelHeight) {
      fprintf(stderr, "Error! The glViewport returned incorrect values! Something is wrong!\n");
      eglDestroyContext(display, context);
      eglDestroySurface(display, surface);
      eglTerminate(display);
      gbmClean();
      return EXIT_FAILURE;
   }

   //   if (seteuid((uid_t)TARGET_UID) == -1) {
   //      fprintf(stderr, "Insufficient user privileges.\n");
   //      return EXIT_FAILURE;
   //   }
   //   
   //   if (setegid((gid_t)TARGET_GID) == -1) {
   //      fprintf(stderr, "Insufficient group privileges.\n");
   //      return EXIT_FAILURE;
   //   }

   lo_server_thread_add_method(st, "/quit", "", quit_handler, NULL);

   lo_server_thread_add_method(st, "/lc4/brightness", NULL, brightness_handler, canvas);

   lo_server_thread_add_method(st, "/lc4/exponent", NULL, exponent_handler, NULL);

   lo_server_thread_add_method(st, "/lc4/x", NULL, x_handler, NULL);
   lo_server_thread_add_method(st, "/lc4/y", NULL, y_handler, NULL);

   lo_server_thread_add_method(st, "/lc4/shader", NULL, shader_handler, NULL);

   lo_server_thread_add_method(st, "/lc4/red", NULL, red_handler, NULL);
   lo_server_thread_add_method(st, "/lc4/green", NULL, green_handler, NULL);
   lo_server_thread_add_method(st, "/lc4/blue", NULL, blue_handler, NULL);
   lo_server_thread_add_method(st, "/lc4/white", NULL, white_handler, NULL);
   lo_server_thread_add_method(st, "/lc4/clear", NULL, clear_handler, NULL);

   lo_server_thread_start(st);

   std::cout << "listening on udp port " << port << std::endl;


   //unsigned char inpacket[4];
   // first open the sequencer device for reading
   //int seqfd = open(MIDI_DEVICE, O_RDONLY);
   //if (seqfd == -1) {
   //printf("Error: cannot open %s\n", MIDI_DEVICE);
   //printf("errno: %d\n", errno);
   //exit(1);
   //}

   glEnable(GL_DEPTH_TEST); 



   float vertices[] = {
      // first triangle
      1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
      1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
      -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
                           // second triangle
      -1.0f,  1.0f, 0.0f,  0.0f, 1.0f,
      -1.0f, -1.0f, 0.0f,  0.0f, 0.0f,
      1.0f,  1.0f, 0.0f,  0.0f, 0.0f
   }; 

   unsigned int indices[] = {  // note that we start from 0!
      2, 3, 0,   // first triangle
      0, 1, 2    // second triangle
   };  

   // Create VBO
   glGenVertexArrays(1, &vao);
   glGenBuffers(1, &vbo);

   //glGenBuffers(1, &ebo);

   glBindVertexArray(vao);
   //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
   //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

   glBindBuffer(GL_ARRAY_BUFFER, vbo);
   glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

   //   // Get vertex attribute and uniform locations
   //   //posLoc = glGetAttribLocation(program, "aPos");
   //   //texLoc = glGetAttribLocation(program, "aTexCoord");
   //
   //
   //   // position attribute
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
   glEnableVertexAttribArray(0);

   // texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

   // load and create a texture 
   // -------------------------
    unsigned int texture;
    // texture
    // ---------
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture); 
     // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   test = (float *)malloc(64*64*sizeof(float));
   for (int i = 0; i < 64; i++) {
      for (int j = 0; j < 64; j++) {
         test[i*64 + j] = 1.0;
      }
   }
   updateTexture();
   //glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f);
   //glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
   //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   unsigned char *buffer = (unsigned char *)malloc(panelWidth * panelHeight * 3);
   std::chrono::time_point<std::chrono::steady_clock> startClock = std::chrono::steady_clock::now();    
   canvas->SetBrightness(brightness);
   int inotifyFd, wd;

   inotifyFd = inotify_init();
   if (inotifyFd == -1) {
      exit(1);
   }

   int flags = fcntl(inotifyFd, F_GETFL);
   if (fcntl(inotifyFd, F_SETFL, flags | O_NONBLOCK) == -1) {
      fprintf(stderr, "fcntl(F_SETFL)");
   }
   wd = inotify_add_watch(inotifyFd, "fence4.frag", IN_MODIFY);
   if (wd == -1) {
      fprintf(stderr, "inotify_add_watch");
   }
   struct inotify_event *event = NULL;
   bool first = true;
   ShaderGL shader1("texture.vs", "fence1.frag");
   ShaderGL shader2("texture.vs", "fence2.frag");
   ShaderGL shader3("texture.vs", "fence3.frag");
   ShaderGL shader4("texture.vs", "fence4.frag");
   ShaderGL shader_render = shader1;
   //PlayMusicStream(music);
   //AttachAudioStreamProcessor(music.stream, audioCallback);
   PaError err = Pa_StartStream(stream);
   checkPAErr(err);
   while(!interrupt_received) {
      updateTexture();
      // Bind the texture and use it in the shader
      
      //UpdateMusicStream(music);
      //printf("\n");

      //		read(seqfd, &inpacket, sizeof(inpacket));
      //		if (inpacket[0] == MIDI_NOTEON) {
      //			DrawOnCanvas(canvas, inpacket[1], inpacket[2]);
      //			//printf("Midi note: %d Velocity: %d\n", inpacket[1], inpacket[2]);
      //			usleep(1*1000);
      //		}

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      struct timespec start;
      timespec_get(&start, TIME_UTC);
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

      // bind textures on corresponding texture units
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture);

      std::chrono::time_point<std::chrono::steady_clock> endClock = std::chrono::steady_clock::now();

      std::chrono::steady_clock::duration time_span = endClock - startClock; 
      float nseconds = float(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
      // get matrix's uniform location and set matrix
      switch (shader) {
         case 1:
            shader_render = shader1;
            break;
         case 2:
            shader_render = shader2;
            break;
         case 3:
            shader_render = shader3;
            shader_render.setFloat("u_exponent", exponent);
            break;
         case 4:
            shader_render = shader4;
            shader_render.setFloat("u_exponent", exponent);
            break;
      }
      char buf[BUF_LEN] __attribute__((aligned(8)));
      int n = read(inotifyFd, buf, BUF_LEN);
      char* p = buf;

      if(p < buf + n) {
         event = (struct inotify_event*)p;
         uint32_t mask = event->mask;
         if (mask & IN_MODIFY && !first) {
            first = true;
            ShaderGL newShader("texture.vs", "fence4.frag");
            shader_render = newShader;
            shader4 = newShader;
            shader = 4;
            // printf("File has been modified\n");
            // std::ifstream f("fence4.frag");
            // if (f.is_open()) {
            //    std::cout << f.rdbuf();
            //    f.seekg(0);
            // }
         }
         else {
            first = false;
         }
      }

      shader_render.use();
      shader_render.setVec2("u_resolution", glm::vec2(64.0f, 64.0f));
      shader_render.setFloat("u_time", nseconds);
      shader_render.setFloat("u_x", x);
      shader_render.setFloat("u_y", y);
      

      
      
      glBindVertexArray(vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      glReadPixels(0, 0, panelWidth, panelHeight, GL_RGB, GL_UNSIGNED_BYTE, buffer);

      for (int y = 0; y < canvas->height(); y++) {
         for (int x = 0; x < canvas->width(); x++) {
            uint8_t *p = &buffer[(y * canvas->width() + x) * 3];
            uint8_t r = *(p+0), g = *(p+1), b = *(p+2);
            canvas->SetPixel(x, y, r, g, b);
         }
      }

      struct timespec end;
      timespec_get(&end, TIME_UTC);
      long tudiff = (end.tv_nsec / 1000 + end.tv_sec * 1000000) - (start.tv_nsec / 1000 + start.tv_sec * 1000000);
      if (tudiff < 10000001 / FPS) {
         usleep(10000001 / FPS - tudiff);
      }
   }
   err = Pa_StopStream(stream);
   checkPAErr(err);

   err = Pa_Terminate();
   fftw_destroy_plan(spectroData->p);
   fftw_free(spectroData->in);
   fftw_free(spectroData->out);
   free(spectroData);
   printf("\n");
   checkPAErr(err);
   free(buffer);
   free(test);
   stbi_image_free(data);
   glDeleteVertexArrays(1, &vao);
   glDeleteBuffers(1, &vbo); 
   eglDestroyContext(display, context);
   eglDestroySurface(display, surface);
   eglTerminate(display);
   gbmClean();
   close(device);
   canvas->Clear();
   close(serial_port);
   delete canvas;
   lo_server_thread_free(st);
   return EXIT_SUCCESS;
}


void error(int num, const char *msg, const char *path)
{
   printf("liblo server error %d in path %s: %s\n", num, path, msg);
   fflush(stdout);
}

int brightness_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *canvas) {
   brightness = argv[0]->f;
   //std::cout << "brightness_handler called: " << brightness << std::endl;
   ((RGBMatrix *)canvas)->SetBrightness(brightness);
   return 0;
}

int exponent_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *user_data) {
   exponent = argv[0]->f;
   //std::cout << "exponent_handler called: " << exponent << std::endl;
   return 0;
}

int x_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *user_data) {
   x = argv[0]->f;
   return 0;
}

int y_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *user_data) {
   y = argv[0]->f;
   return 0;
}

int shader_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *user_data) {
   shader = argv[0]->f;
   return 0;
}

int red_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
   red = argv[0]->f;
   int redInt = static_cast<int>(red);
   //std::cout << "red_handler called: " << *((GLint *)colorLoc) << std::endl;
   //glUniform4f(*((GLint *)colorLoc), red, green, blue, 1.0f);
   std::string msg = "r:" + std::to_string(redInt);
   char r[5];
   strncpy(r, msg.c_str(), sizeof(r));
   //std::cout << msg << std::endl;
   write(serial_port, r, msg.length());
   return 0;
}


int green_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
   green = argv[0]->f;
   int greenInt = static_cast<int>(green);
   std::string msg = "g:" + std::to_string(greenInt);
   char g[5];
   strncpy(g, msg.c_str(), sizeof(g));
   //std::cout << msg << std::endl;
   write(serial_port, g, msg.length());
   return 0;
}


int blue_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
   blue = argv[0]->f;
   int blueInt = static_cast<int>(blue);
   std::string msg = "b:" + std::to_string(blueInt);
   char b[5];
   strncpy(b, msg.c_str(), sizeof(b));
   //std::cout << msg << std::endl;
   write(serial_port, b, msg.length());
   return 0;
}

int white_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
   white = argv[0]->f;
   int whiteInt = static_cast<int>(white);
   std::string msg = "w:" + std::to_string(whiteInt);
   char w[5];
   strncpy(w, msg.c_str(), sizeof(w));
   //std::cout << msg << std::endl;
   write(serial_port, w, msg.length());
   return 0;
}

int clear_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
   unsigned char c[] = {'c'};
   write(serial_port, c, 1);
   //glUniform4f(*((GLint *)colorLoc), red, green, blue, 1.0f);
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


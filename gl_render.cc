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

#include <string.h>
#include <sys/soundcard.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <lo/lo.h>
#include <lo/lo_cpp.h>
#include <cmath>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <fcntl.h>
#include <unistd.h>
#include "led-matrix.h"
#include <time.h>
#include "shader.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include <time.h>
#include <sys/time.h>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define MIDI_DEVICE "/dev/sequencer"
#define FPS 160
int done = 0;
int count = 0;
int brightness = 0;
float red = 0.0;
float green = 0.0;
float blue = 0.0;
int panelWidth = 64;
int panelHeight = 64;
float rotation = 0;

int device;
drmModeModeInfo mode;
struct gbm_device *gbmDevice;
struct gbm_surface *gbmSurface;
drmModeCrtc *crtc;
RGBMatrix * canvas;
uint32_t connectorId;

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
int rotate_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data);


volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
	interrupt_received = true;
}

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

int main(int argc, char *argv[]) {

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
   GLuint program, vert, frag, vbo, vao, ebo;
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

   lo_server_thread_add_method(st, "/lc4/rotate", NULL, rotate_handler, NULL);
	
	//lo_server_thread_add_method(st, "/lc4/red", NULL, red_handler, &colorLoc);
	//lo_server_thread_add_method(st, "/lc4/green", NULL, green_handler, &colorLoc);
	//lo_server_thread_add_method(st, "/lc4/blue", NULL, blue_handler, &colorLoc);

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
   
   Shader shader("texture.vs", "texture.fs");

   float vertices[] = {
    // positions          // colors           // texture coords
     0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
     0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
    -0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
    -0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left 
   };
    unsigned int indices[] = {  
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };
    // Create VBO
   glGenVertexArrays(1, &vao);
   glGenBuffers(1, &vbo);
   glGenBuffers(1, &ebo);

   glBindVertexArray(vao);

   glBindBuffer(GL_ARRAY_BUFFER, vbo);
   glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
   glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

   // Get vertex attribute and uniform locations
   //posLoc = glGetAttribLocation(program, "aPos");
   //texLoc = glGetAttribLocation(program, "aTexCoord");
   

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
   
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
    // load image, create texture and generate mipmaps
    int width, height, nrChannels;
    //stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
    unsigned char *data = stbi_load("./resources/textures/awesomeface.png", &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        std::cout << "Loaded Container\n";
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);


   //glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f);
   //glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
   //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   unsigned char *buffer = (unsigned char *)malloc(panelWidth * panelHeight * 3);
   std::chrono::time_point<std::chrono::steady_clock> startClock = std::chrono::steady_clock::now();    
   canvas->SetBrightness(brightness);
   while(!interrupt_received) {
      
//		read(seqfd, &inpacket, sizeof(inpacket));
//		if (inpacket[0] == MIDI_NOTEON) {
//			DrawOnCanvas(canvas, inpacket[1], inpacket[2]);
//			//printf("Midi note: %d Velocity: %d\n", inpacket[1], inpacket[2]);
//			usleep(1*1000);
//		}
      struct timespec start;
      timespec_get(&start, TIME_UTC);
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      //glUniform4f((colorLoc), red, green, blue, 1.0f);

      // bind textures on corresponding texture units
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture);

      // create transformations
      glm::mat4 transform = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
      transform = glm::translate(transform, glm::vec3(0.0f, 0.0f, 0.0f));
      transform = glm::rotate(transform, rotation, glm::vec3(0.0f, 0.0f, 1.0f));

      // get matrix's uniform location and set matrix
      shader.use();
      unsigned int transformLoc = glGetUniformLocation(shader.ID, "transform");
      glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

      glBindVertexArray(vao);
      glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
      
      // Create buffer to hold entire front buffer pixels

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
   free(buffer);
   glDeleteVertexArrays(1, &vao);
   glDeleteBuffers(1, &vbo);
   glDeleteBuffers(1, &ebo);  
   eglDestroyContext(display, context);
   eglDestroySurface(display, surface);
   eglTerminate(display);
   gbmClean();
   close(device);
	canvas->Clear();
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
   std::cout << "brightness_handler called: " << brightness << std::endl;
	((RGBMatrix *)canvas)->SetBrightness(brightness);
	return 0;
}

int rotate_handler(const char *path, const char *types, lo_arg ** argv,
		int argc, lo_message data, void *user_data) {
         rotation = argv[0]->f;
         return 0;
      }

int red_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
	red = argv[0]->f;
	red /= 255.0;
   //std::cout << "red_handler called: " << *((GLint *)colorLoc) << std::endl;
   glUniform4f(*((GLint *)colorLoc), red, green, blue, 1.0f);
	return 0;
}


int green_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
	green = (float) argv[0]->i;
	green /= 255.0;
   glUniform4f(*((GLint *)colorLoc), red, green, blue, 1.0f);
	return 0;
}


int blue_handler(const char *path, const char *types, lo_arg ** argv, int argc, lo_message data, void *colorLoc) {
	blue = (float) argv[0]->i;
	blue /= 255.0;
   glUniform4f(*((GLint *)colorLoc), red, green, blue, 1.0f);
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


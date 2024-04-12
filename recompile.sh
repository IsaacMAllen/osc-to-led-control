#! /bin/bash
g++ -Imatrix/include -I/usr/include/libdrm -I/usr/include/GLES3 -Wall -O3 -g -c -o gl_render.o gl_render.cc 
g++ -Wall -O3 -g gl_render.o -o gl_render -L/usr/local/lib -Lmatrix/lib -lrgbmatrix -lrt -lm -pthread -llo -lGL -lEGL -lGLESv2 -ldrm -lgbm -lraylib -latomic -lfftw3 -lasound -ljack -lportaudio
sudo install -o root -g root -m u=rxs,g=rx,o=x -t ./bin gl_render

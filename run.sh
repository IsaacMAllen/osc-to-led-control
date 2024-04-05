#! /bin/bash
xhost + local:
./bin/gl_render --led-rows=32 --led-cols=64 --led-chain=2 --led-gpio-mapping=adafruit-hat-pwm --led-pixel-mapper="V-mapper;Rotate:90;Mirror:V" --led-panel-type=FM6126A --led-slowdown-gpio=5

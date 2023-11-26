#!/bin/bash

g++ -g ./src/*.cpp -I./include/ `pkg-config --cflags --libs opencv4` -o ldlidar
chmod +x ldlidar

echo "compiled successfully"
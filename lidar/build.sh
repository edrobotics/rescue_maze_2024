#!/bin/bash

g++ -g -std=gnu++17 -pedantic -Wall -Wextra `find src -iregex ".*\.cpp"` -I./include/ `pkg-config --cflags --libs opencv4` -o ldlidar -Og #-Ofast #use O3 or O2 if not working
chmod +x ldlidar

echo "compiled successfully (maybe)"
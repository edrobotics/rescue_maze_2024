#!/bin/bash

g++ -g -std=gnu++17 `find src -iregex ".*\.cpp"` -I./include/ `pkg-config --cflags --libs opencv4` -o ldlidar #-O3 #use O3 or O2 if not working
chmod +x ldlidar

echo "compiled successfully (maybe)"
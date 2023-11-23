#!/bin/bash
# change pls
echo "Fix this script idiot"
g++ -g ./src/*.cpp -I./include/ -I/usr/include/opencv4/ -o ld_test -D LINUX -L/home/markfri/code/opencv/build/lib/ -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc #-LC:/msys64/ucrt64/lib/ -L./lib/ -lsetupapi -lopencv_core470 -lopencv_imgcodecs470 -lopencv_imgproc470

echo "compiled successfully"
#!/bin/bash

g++ -g -std=gnu++17 ./src/*.cpp ./src/*/*.cpp -I./include/ -o ./build/mazenav -Og #use O3 or O2 if not working (previously Ofast)
chmod +x ./build/mazenav

echo "compiled successfully (maybe)"
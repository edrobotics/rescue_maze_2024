#!/bin/bash

g++ -g -std=gnu++17 ./src/*.cpp -I./include/ -o mazenav -Ofast #use O3 or O2 if not working
chmod +x mazenav

echo "compiled successfully (maybe)"
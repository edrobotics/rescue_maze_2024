#!/bin/bash

env=''
envDefine=''
wiringPiInclude=''

while getopts 'e:' flag; do
    case "${flag}" in
        e) env="${OPTARG}" ;;
    esac
done

echo $env

if [ $env = 'pi' ]; then
    envDefine='-DENV_PI'
    wiringPiInclude='-lwiringPi'
elif [ $env = 'dev' ]; then
    envDefine='-DENV_DEV'
else
    printf "Error: Unknown environment: $env\nOptions: -e pi/dev\nRun command again with correct environment\n"
    exit 1
fi

# rm -R ./build/
mkdir ./build

g++ -g -std=gnu++17 -pedantic -Wall -Wextra `find src -iregex ".*\.cpp"` `find ../shared_lib -iregex ".*\.cpp"` -I./include/ -I../shared_lib/ $wiringPiInclude -o ./build/mazenav -Og $envDefine #use O3 or O2 if not working (previously Ofast)

if [ $? -eq 1 ]; then
    echo "Could not build (compile/link/other dark magic)... :("
else
    echo "Compiled successfully :)"
fi

chmod +x ./build/mazenav
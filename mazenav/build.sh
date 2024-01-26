#!/bin/bash

env=''
envDefine=''


while getopts 'e:' flag; do
    case "${flag}" in
        e) env="${OPTARG}" ;;
    esac
done

echo $env

if [ $env = 'pi' ]; then
    envDefine='-DENV_PI'
elif [ $env = 'dev' ]; then
    envDefine='-DENV_DEV'
else
    printf "Error: Unknown environment: $env\nOptions: -e pi/dev\nRun command again with correct environment\n"
    exit 1
fi


g++ -g -std=gnu++17 -pedantic -Wall -Wextra `find src -iregex ".*\.cpp"` `find ../shared_lib -iregex ".*\.cpp"` -I./include/ -I../shared_lib/ -I/usr/local/boost_1_82_0/ -o ./build/mazenav -Og $envDefine #use O3 or O2 if not working (previously Ofast)
chmod +x ./build/mazenav

echo "compiled successfully (maybe)"
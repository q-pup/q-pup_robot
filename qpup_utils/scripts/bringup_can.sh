#!/bin/bash

#TODO: port to python script via subprocess?

if sudo ip link set can0 type can bitrate 1000000; then
     echo "Successfully setup can interface: can0. Bitrate: 1000000"
else
     echo "Failure, exit status: $?"
fi

if sudo ip link set up can0; then
     echo "Successfully brought up can interface: can0"
else
     echo "Failure, exit status: $?"
fi

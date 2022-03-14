#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000

if sudo ip link set can0 txqueuelen 10000; then
  echo "Successfully set can0 interface to txqueuelen 10000"
else
  echo "Failure, exit status: $?"
fi

if sudo ip link set can1 txqueuelen 10000; then
  echo "Successfully set can1 interface to txqueuelen 10000"
else
  echo "Failure, exit status: $?"
fi

sudo ip link set up can0
sudo ip link set up can1

exit 0
#!/bin/bash

if [ $# -eq 0 ]
	then
		echo "device name required, exiting..."
else
	sudo modprobe peak_usb
	sudo ip link set $1 type can bitrate 1000000 restart-ms 10
	sudo ip link set $1 txqueuelen 256
	sudo ip link set up $1
fi

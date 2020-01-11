#!/bin/bash

echo "Starting soft serial"
sudo pigpiod

sleep 5

echo "Starting the butterflyfeeder"
#python /home/pi/butterflyfeeder/bfeeder.py &

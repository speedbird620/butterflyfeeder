#!/bin/bash

echo "Startar IO-driver"
sudo pigpiod

sleep 5

echo "Startar ADSB handler"
python /home/pi/bfeeder/bfeeder.py &

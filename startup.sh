#!/bin/bash
#
# Please enter the following in crontab-e:
# @reboot sudo sh /home/pi/butterflyfeeder/startup.sh
#

echo "Starting soft serial"
sudo pigpiod

sleep 5

echo "Starting the butterflyfeeder"
python3 /home/pi/butterflyfeeder/bfeeder.py &

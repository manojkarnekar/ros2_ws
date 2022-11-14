#!/bin/bash

ssh pi@192.168.10.101 sh /home/pi/server_pi/scripts/killall.sh

/home/dev/sar/utilities/scripts/killall.sh

echo "irobot" | sudo -S shutdown -h now
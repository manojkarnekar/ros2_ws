#!/bin/bash

ssh pi@192.168.10.101 sh /home/pi/server_pi/scripts/restart.sh

echo "irobot" | sudo -S reboot
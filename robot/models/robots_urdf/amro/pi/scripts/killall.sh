#!/bin/bash

cd /home/pi/server_pi/scripts/
sudo python TabOFF.py
echo "TabOFF done"

echo "PI is shutting Down"
sudo shutdown -h now



#!/bin/bash

cd /home/pi/server_pi/
sudo screen -S sar.db -d -m node index.js

cd /home/pi/server_pi/scripts/
sudo python TabON.py



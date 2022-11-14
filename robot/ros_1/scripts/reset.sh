#!/bin/bash

/home/dev/sar/utilities/scripts/killall.sh

pkill screen

screen -dmS sar.runner sh -c "/home/dev/sar/utilities/scripts/run.sh -module amro -system N2 -sleep 20"

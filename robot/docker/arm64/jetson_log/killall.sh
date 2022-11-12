#!/bin/bash

echo "Shutting Screens Down.."

for sar in `screen -list | grep -oh "[0-9]*.sar.[a-z]*"`; do 
	screen -S $sar -X at "#" stuff ^C
done

echo "Done"
rosclean purge -y
echo "rosclean done"

sleep 5

pkill screen

screen -ls

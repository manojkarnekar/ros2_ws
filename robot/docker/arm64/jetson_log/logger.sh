#!/bin/bash

# sudo apt install screen
echo "docker stats and jtop data logger"
screen -dmS sar.docker_logger docker stats --format "table {{.Name}},{{.CPUPerc}},{{.MemPerc}},{{.MemUsage}}" | tee --append data/docker_stats.csv
screen -dmS sar.jtop_logger python3 jetson_logger.py
screen -ls
#!/bin/bash
killall gzserver
killall gzclient
killall -9 gazebo
ps aux | grep gazebo | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
rm -rf ~/.gazebo/models/four_wheeled_robot
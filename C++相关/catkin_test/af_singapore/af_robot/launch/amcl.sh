#!/bin/bash

rosnode kill /slam_gmapping
rosnode kill /amcl
rosnode kill /amcl_select
gnome-terminal -x bash -c "roslaunch af_nav amcl_map.launch"

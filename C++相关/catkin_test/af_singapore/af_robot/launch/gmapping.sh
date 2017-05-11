#!/bin/bash

rosnode kill /amcl_tf
rosnode kill /amcl_select
rosnode kill /amcl
gnome-terminal -x bash -c "roslaunch af_nav gmapping.launch.xml"

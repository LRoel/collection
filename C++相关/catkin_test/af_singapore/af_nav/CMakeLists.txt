cmake_minimum_required(VERSION 2.8.3)
project(af_nav)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS tf roscpp sensor_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS tf roscpp sensor_msgs
)



###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# Add_executables


#############
## Install ##
#############

# Mark executables and/or libraries for installation

# Mark anything (useful) else for installation
install(DIRECTORY laser
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Intro

The repo is an offboard control example for Kerloud Auto Car Series. It can serve as the basis for further application development based on C++.

Kerloud Auto Car series: <https://kerloud-autocar.readthedocs.io>

Tutorial guide: <https://kerloud-autocar.readthedocs.io/en/latest/tutorials/offboard_cplus.html>

# Environment Requirements

Recommended environment: Ubuntu 18.04, ROS melodic


Prerequisites:

* mavros (dev_rover branch): https://github.com/cloudkernel-tech/mavros
* mavlink (dev_rover branch): https://github.com/cloudkernel-tech/mavlink-gdp-release


# How to run

## Simulation test

        # launch a terminal
        cd ~/src/rover_workspace/catkinws_offboard
        source devel/setup.bash
        roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

        # launch a new terminal
        cd ~/src/rover_workspace/catkinws_offboard
        source devel/setup.bash
        roslaunch off_mission off_mission.launch simulation_flag:=1


## Real test

        # launch a terminal
        cd ~/src/rover_workspace/catkinws_offboard
        source devel/setup.bash
        roslaunch mavros px4.launch fcu_url:="/dev/ttyPixhawk:921600"

        # launch a new terminal
        cd ~/src/rover_workspace/catkinws_offboard
        source devel/setup.bash
        roslaunch off_mission off_mission.launch

# bgt60tr13c_ros
ROS package for the bgt60ltr13c radar from Infineon


## Setup
In order to get this radar working on linx you need to have access to the radar_sdk from Infineon.
This can be obtained by signing up on their website for an account, downloading the infineon developer center, and then installing the tool "radar development toolbox" on windows. Inside of the directory where this toolbox is installed there should be a directory called radar_sdk which you need to copy onto your linux system.

Once you have it there you create a docker container or use the one referenced in the main repo https://github.com/AutonomousVehicleLaboratory/sensor_platform/tree/main
This image should have all the necessary dependencies to install the sdk

Then inside of this container you need to follow the instructions in the documentation within the radar_sdk/doc to build the sdk on linux.

1. Inside of radar_sdk folder run the following commands
* `mkdir build && cd build`
* `cmake ..`
* `make`
* `make install`
* `cmake --build . --target wheel-avian`
2. Then navigate to the new directory wheel_avian/dist and run
* `python3 -m pip install ifxAvian-3.4.0-py3-none-any.whl`

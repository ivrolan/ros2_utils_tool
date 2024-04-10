## Overview

The mediassist4_ros_tools package provides various tools for the everyday-usage of ROS2. The package provides UI as well as command line functionality. Every tool can be used inside the UI as well as the command line.\ The tool provides the following functionalities:
- Encode a video out of a ROSBag to mp4 or mkv\
NOTE: The package is still under active development, so more tools might be added later in the future.

## Installation:

### Dependencies

The following packages are required:
- ROS2 for all ROS-related functionalities. The tools run on version _humble_.
- OpenCV for writing video files.
- Qt5/Qt6 for all UI as well as some convenience functionalities.

Take a look at [1] for the installation of ROS2 Humble on Ubuntu 22.04.\
To install OpenCV on Ubuntu 22.04, use the following command:
```
sudo apt install libopencv-dev
```

For Qt5 and/or Qt6:
```
sudo apt install qtbase5-dev qt6-base-dev
```

### Build the tool

1. Navigate to your ROS2 workspace's src direction:
```
cd path/to/your/workspace/src
```

2. Clone this repository:
```
git clone https://gitlab.com/nct_tso_public/mediassist4/mediassist4_ros_tools.git
```

3. Navigate back to your workspace:
```
cd path/to/your/workspace/
```

4. Build the project and source it:
```
colcon build
source install/setup.bash
```

## Usage

To start the UI, use the following command:
```
ros2 run mediassist4_ros_tools tool_app
```

Start the video-to-bag-tool with the following command:
```
ros2 run mediassist4_ros_tools tool_bag_to_video /path/to/bag_file topic_name_of_video_topic /path/where/video/should/be/stored use_hardware_acceleration
```
(The video needs to have an .mp4 or .mkv appendix, the hardware acceleration parameter needs to be `true` or `false`.

[1] https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

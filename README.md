## Overview

The mediassist4_ros_tools package provides various tools for the everyday-usage of ROS2. The package provides a UI as well as command line functionality.

As of now, the tool provides the following functionalities:
- Encode a video out of a ROSBag video topic to mp4 or mkv
- Write images of of a ROSBag video topic, either using jpg or png format
- Write an mp4 or mkv video file to a ROSBag.
- Create a dummy bag file, containing either string, int or image messages

NOTE: The package is still under active development, so more tools might be added later in the future.

## Installation:

### Dependencies

The following packages are required:
- ROS2 for all ROS-related functionalities. The tools run on version **humble**.
- OpenCV for writing video files.
- The ROS-CV-Bridge for converting ROS sensor images to cv matrices and vice versa.
- Qt5/Qt6 for all UI as well as some convenience functionalities.
- (Optional) Catch2-ROS for Catch2-based unit tests with ROS2.

Take a look at [1] for the installation of ROS2 Humble on Ubuntu 22.04.

**OpenCV**:
```
sudo apt install libopencv-dev
```

**ROS-CV-Bridge:**
```
ros-humble-cv-bridge
```

**Qt5/Qt6**:
```
sudo apt install qtbase5-dev qt6-base-dev
```

**Catch2 for ROS2**:
```
sudo apt install ros-humble-catch-ros2
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

**Full UI**:
```
ros2 run mediassist4_ros_tools tool_ui
```

**Bag-to-Video-Tool**:
```
ros2 run mediassist4_ros_tools tool_bag_to_video /path/to/bag_file topic_name_of_video_topic /path/where/video/should/be/stored
```
(The video needs to have an .mp4 or .mkv appendix).

**Bag-to-Images-Tool**:
```
ros2 run mediassist4_ros_tools tool_bag_to_images /path/to/bag_file topic_name_of_video_topic /path/where/images/should/be/stored --format image_format
```
(`image_format` needs to be either `jpg`, `png` or `bmp`).

**Video-to-Bag-Tool**:
```
ros2 run mediassist4_ros_tools tool_video_to_bag /path/to/video_file topic_name_in_bag /path/where/bag/should/be/stored
```
(The video needs to have an .mp4 or .mkv appendix).

**Dummy-Bag-Tool**:
```
ros2 run mediassist4_ros_tools tool_dummy_bag path/to/ROSBag topic_type_1 topic_name_1 ... message_count
```
(Topic type needs to be `String`, `Integer` or `Image`, up to three topics can be written, `message_count` needs to be a value from 1 to 1000).

**Unit tests**:
```
ros2 run mediassist4_ros_tools tool_tests
```

[1] https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Overview

The ros2_utils_tool package provides various tools for the everyday-usage of ROS2. The package provides a UI as well as command line functionality.

As of now, the tool provides the following functionalities:
- Encode a video out of a ROS bag video topic to mp4 or mkv (uncompressed is also supported for mkv)
- Write images out of a ROS bag video topic, either using jpg, bmp or png format
- Write an mp4 or mkv video file to a ROS bag.
- Create a dummy ROS bag file, containing either string, int or image messages
- Get information for a ROS bag (UI only)

NOTE: The package is still under active development, so more tools might be added later in the future.

## Installation:

### Dependencies

The following packages are required:
- [ROS2](https://docs.ros.org/en/jazzy/index.html), both version **humble** and **jazzy** are supported.
- [OpenCV](https://opencv.org/) for writing video files.
- [cv_bridge](https://index.ros.org/p/cv_bridge/) for converting ROS sensor images to cv matrices and vice versa.
- [Qt5/Qt6](https://doc.qt.io/) for all UI as well as some convenience functionalities.
    - The application uses Qt6 by default. If no Qt6 is found, Qt5 is used instead.
- (Optional) [catch2_ROS](https://index.ros.org/p/catch_ros2/) for Catch2-based unit tests with ROS2.

**OpenCV**:\
`sudo apt install libopencv-dev`

**ROS-CV-Bridge:**\
`sudo apt install ros-humble-cv-bridge` or `sudo apt install ros-jazzy-cv-bridge`

**Qt5/Qt6**:\
`sudo apt install qtbase5-dev qt6-base-dev`

**Catch2 for ROS2**:\
`sudo apt install ros-humble-catch-ros2` or `sudo apt install ros-jazzy-catch-ros2`

### Build the tool

1. Navigate to your ROS2 workspace's src direction:
```
cd path/to/your/workspace/src
```

2. Clone this repository:
```
git clone https://gitlab.com/nct_tso_public/mediassist4/ros2_utils_tool.git
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
ros2 run ros2_utils_tool tool_ui
```

**Bag-to-Video-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_video /path/to/bag_file topic_name_of_video_topic /path/where/video/should/be/stored
```
(The video needs to have an .mp4 or .mkv appendix).

**Bag-to-Images-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_images /path/to/bag_file topic_name_of_video_topic /path/where/images/should/be/stored --format image_format
```
(`image_format` needs to be either `jpg`, `bmp` or `png`).

**Video-to-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_video_to_bag /path/to/video_file topic_name_in_bag /path/where/bag/should/be/stored
```
(The video needs to have an .mp4 or .mkv appendix).

**Dummy-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_dummy_bag path/to/ROSBag topic_type_1 topic_name_1 ... message_count
```
(Topic type needs to be `String`, `Integer` or `Image`, up to three topics can be written, `message_count` needs to be a value from 1 to 1000).

**Unit tests**:
```
ros2 run ros2_utils_tool tool_tests
```

## License

The ros2_utils_tool package is licensed under [EUPLv1.2](https://interoperable-europe.ec.europa.eu/sites/default/files/custom-page/attachment/2020-03/EUPL-1.2%20EN.txt).

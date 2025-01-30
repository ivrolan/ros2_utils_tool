<div align="center">

   ![License badge](https://img.shields.io/badge/License-EUPLv1.2-blue.svg)
   ![C++ badge](https://img.shields.io/badge/C++-20-blue.svg)
   ![CI Ubuntu badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/humble.yml/badge.svg?event=push)
   ![CI Windows badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/jazzy.yml/badge.svg?event=push)
   ![Tag badge](https://img.shields.io/badge/Release-v0.7.0-blue.svg)

</div>

## Overview

The ros2_utils_tool package provides various tools for the everyday-usage of ROS2. The package provides a UI as well as command line functionality.

As of now, the tool provides the following functionalities:
- Encode a video out of a ROS bag video topic to mp4 or mkv (uncompressed is also supported for mkv)
- Write a video to a ROS bag
- Write images out of a ROS bag video topic
- Create a dummy ROS bag file, containing either string, int or image messages
- Edit an existing ROS bag file, with support for renaming or dropping topics and changing the message count (UI only)
- Get information for a ROS bag (UI only)
- Publish a video as a ROS topic
- Publish images as a ROS topic

The following table will give an overview which functionalities are currently support as a user interface and/or command line interface tool:

| Feature  | UI support | CLI support |
| -------- | ---------- | ----------- |
| ROS bag video topic to video |  X  |  X  |
| Video file to ROS bag |  X  |  X  |
| ROS bag video topic to images |  X  |  X  |
| Create dummy bag ROS file |  X  |  X  |
| Edit ROS bag file |  X  |    |
| UI-based ROS bag file information |  X  |    |
| Publish Video as ROS Topic |  X  |  X  |
| Publish Images as ROS Topic |  X  |  X  |

NOTE: The package is still under active development, so more tools might be added later in the future. Additionally, already existing features might expand and change constantly.

## Installation:

### Dependencies

The following packages are required:
- [ROS2](https://docs.ros.org/en/jazzy/index.html), both version **humble** and **jazzy** are supported.
- [OpenCV](https://opencv.org/) for writing video files.
- [cv_bridge](https://index.ros.org/p/cv_bridge/) for converting ROS sensor images to cv matrices and vice versa.
- [Qt5/Qt6](https://doc.qt.io/) for all UI as well as some convenience functionalities.
    - The application uses Qt6 by default. If no Qt6 is found, Qt5 is used instead.
- [catch2_ROS](https://index.ros.org/p/catch_ros2/) for Catch2-based unit tests with ROS2.
- (Optional) [uncrustify](https://github.com/uncrustify/uncrustify) for code formatting.

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
ros2 run ros2_utils_tool tool_bag_to_video /path/to/bag_file /path/where/video/should/be/stored
```
(Note that a topic can be specified optionally. If no topic is specified, the first available video topic is used. The video needs to have an .mp4 or .mkv appendix).

**Video-to-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_video_to_bag /path/to/video_file /path/where/bag/should/be/stored
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. The video needs to have an .mp4 or .mkv appendix).

**Bag-to-Images-Tool**:
```
ros2 run ros2_utils_tool tool_bag_to_images /path/to/bag_file /path/where/images/should/be/stored
```
(Note that a topic can be specified optionally. If no topic is specified, the first available video topic is used. `image_format` needs to be either `jpg`, `bmp` or `png`, jpg is default).

**Dummy-Bag-Tool**:
```
ros2 run ros2_utils_tool tool_dummy_bag path/to/ROSBag topic_type_1 topic_name_1 ... message_count
```
(Topic type needs to be `String`, `Integer` or `Image`, up to three topics can be written, `message_count` needs to be a value from 1 to 1000).

**Publish-Video-Tool**:
```
ros2 run ros2_utils_tool tool_publish_video path/to/video
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. The video needs to have an .mp4 or .mkv appendix).

**Publish-Images-Tool**:
```
ros2 run ros2_utils_tool tool_publish_images path/to/images/dir
```
(Note that a topic can be specified optionally. If no topic is specified, a predefined topic name will be taken. Images need to be of format `jpg`, `bmp` or `png`).

**Unit tests**:
```
ros2 run ros2_utils_tool tool_tests
```

## License

The ros2_utils_tool package is licensed under [EUPLv1.2](https://interoperable-europe.ec.europa.eu/sites/default/files/custom-page/attachment/2020-03/EUPL-1.2%20EN.txt).

## Contribution 

If you discover a new bug or wish for a new feature, feel free to open a new issue.\
If you want to contribute another feature, please use the provided Uncrustify file for code formatting.

<div align="center">

   ![License badge](https://img.shields.io/badge/License-EUPLv1.2-blue.svg)
   ![C++ badge](https://img.shields.io/badge/C++-20-blue.svg)
   ![CI Ubuntu badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/humble.yml/badge.svg?event=push)
   ![CI Windows badge](https://github.com/MaxFleur/ros2_utils_tool/actions/workflows/jazzy.yml/badge.svg?event=push)
   ![Tag badge](https://img.shields.io/badge/Release-v0.7.2-blue.svg)

</div>

<p align="center">
  <img width="360" height="480" src="https://github.com/user-attachments/assets/261c7f8b-ccc5-4734-8edc-81c9e063a4ae">
  <img width="360" height="480" src="https://github.com/user-attachments/assets/219db62e-10ee-4d0a-b712-cc227c91ee97">
</p>
<p align="center">
  <img width="720" height="120" src="https://github.com/user-attachments/assets/b34534c8-c751-4fec-b58e-0643ea0b7f3f">
</p>

# Overview

The ros2_utils_tool package provides a complete UI-based tool for the everyday-usage of ROS2 with additional CLI support. As of now, the tool provides the following functionalities:

| Tool  | Description | UI support | CLI support |
| ----- | ----------- | ---------- | ----------- |
| Bag to Video | Export a ROS bag video topic to a video |  X  |  X  |
| Video to Bag | Port a video file to a ROS bag |  X  |  X  |
| Bag to Images | Export a ROS bag video topic to an image sequence |  X  |  X  |
| Dummy Bag | Create a ROS bag with dummy data |  X  |  X  |
| Edit Bag | Rename, remove or crop topics in a ROS bag |  X  |    |
| UI-based Bag Info | UI-supported bag info vis |  X  | (X) (`ros2 bag info`) |
| Video as ROS Topic | Publish a video file as a ROS image_msg topic |  X  |  X  |
| Image Sequence as ROS Topic | Publish a file with images as ROS image_msg Topic |  X  |  X  |

NOTE: The package is still under active development, so more tools might be added later in the future. Additionally, already existing features might expand and change constantly.

## Installation

### Dependencies

The following packages are required:
- [ROS2](https://docs.ros.org/en/jazzy/index.html), both version **humble** and **jazzy** are supported.
- [OpenCV](https://opencv.org/) for writing video files.
- [cv_bridge](https://index.ros.org/p/cv_bridge/) for converting ROS sensor images to cv matrices and vice versa.
- [Qt6/Qt5](https://doc.qt.io/) for all UI as well as some convenience functionalities.
    - The application uses Qt6 by default. If no Qt6 installation is found on the system, Qt5 is used instead.
    - **Due to Qt5 going end-of-life on May 26th 2025, support for it will be dropped soon**.
- [catch2_ROS](https://index.ros.org/p/catch_ros2/) for Catch2-based unit tests with ROS2.
- (Optional) [uncrustify](https://github.com/uncrustify/uncrustify) for code formatting.

The following command installs all additional dependencies at once:

**Humble**:\
`sudo apt install libopencv-dev ros-humble-cv-bridge qt6-base-dev qtbase5-dev ros-humble-catch-ros2`

**Jazzy**:\
`sudo apt install libopencv-dev ros-jazzy-cv-bridge qt6-base-dev qtbase5-dev ros-jazzy-catch-ros2`

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
If you want to contribute another feature, please use the provided Uncrustify file for code formatting. As the `main` branch is only updated for new versions or critical bugfixes, the `develop` branch is the most current one, providing the newest updates and features. So please open a merge request with `develop` as the target branch.

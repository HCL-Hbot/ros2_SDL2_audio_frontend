# ROS2 Audio Frontend
ROS2 package for audio capture and processing using SDL2, including Voice Activity Detection (VAD) and wake word detection.

## Components
- audio_capture_node: Captures raw audio from microphone using SDL2
- vad_node: Performs Voice Activity Detection
- wake_word_node: Detects wake words in audio stream

## Dependencies
- ROS2 (tested with Jazzy)
- audio_common_msgs
- SDL2

## TODO

### Building
<!-- - [ ] Dowbload and build https://github.com/ros-drivers/audio_common.git as part of cmake
- [ ] Fix error: [ 33%] Building CXX object CMakeFiles/audio_capture_node.dir/src/audio_capture_node.cpp.o
/home/jeroen/ros2_ws/src/edgevox_ros/src/audio_capture_node.cpp:70:10: fatal error: rclcpp_components/register_node_macro.hpp: No such file or directory
   70 | #include "rclcpp_components/register_node_macro.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated. -->
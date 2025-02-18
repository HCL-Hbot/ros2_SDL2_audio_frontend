# ROS2 Audio Frontend
ROS2 package for audio playback, capture and processing using SDL2, including Voice Activity Detection (VAD) and wake word detection.

## Components
- audio_capture_node: Captures raw audio from microphone using SDL2
- audio_playback_node: Plays raw audio to a speaker using SDL2
- vad_node: Performs Voice Activity Detection
- wake_word_node: Detects wake words in audio stream

## Dependencies
- ROS2 (tested with Jazzy)
- SDL2

## TODO

- [ ] Fix error:
ros2 launch sdl2_audio_frontend audio_frontend.launch.py
[INFO] [launch]: All log files can be found below /home/jeroen/.ros/log/2025-02-18-21-40-43-463756-sadalsuud-26487
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [audio_capture_node-1]: process started with pid [26490]
[audio_capture_node-1] /home/jeroen/local_dev/ros2_SDL2_audio_frontend/install/sdl2_audio_frontend/lib/sdl2_audio_frontend/audio_capture_node: error while loading shared libraries: libsdl2_audio_utils.so: cannot open shared object file: No such file or directory
[ERROR] [audio_capture_node-1]: process has died [pid 26490, exit code 127, cmd '/home/jeroen/local_dev/ros2_SDL2_audio_frontend/install/sdl2_audio_frontend/lib/sdl2_audio_frontend/audio_capture_node --ros-args -r __node:=audio_capture_node --params-file /home/jeroen/local_dev/ros2_SDL2_audio_frontend/install/sdl2_audio_frontend/share/sdl2_audio_frontend/config/audio_params.yaml'].

- [ ] audio_playback_node
- [ ] vad_node
- [ ] wake_word_node
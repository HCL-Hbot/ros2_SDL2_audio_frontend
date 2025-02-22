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

- [ ] audio_playback_node
- [ ] vad_node
- [ ] wake_word_node
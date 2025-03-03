# ROS2 Audio Frontend
ROS2 package for audio playback, capture and processing using SDL2, including Voice Activity Detection (VAD) and wake word detection.
The nodes are intended to provide input to conversation manager package.

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
- [ ] More sophisticated voice detection vad_node
- [ ] wake_word_node
- [ ] retreive voice characterisitics for speaker diarization
- [ ] echo cancellation when using audio_playback and audio_capture simultaneously
- [ ] noise suppression
- [ ] more robust wake_word_node that accounts for Dutch speakers, with different dialects 
- [ ] Set all node parameters that consider time in _ms

1. It seems the Lowwi library is trying to find the onnx files in a hardcoded "models" directory. How about providing a environment variable to point to the right path?
2. I am very much in favor of providing params.yaml files to set node parameters
3. MultiThreadedExecutor seems useful for processing queued audio in a separate thread managed by ROS

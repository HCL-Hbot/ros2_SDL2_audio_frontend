# Overview

## Project Structure
```
ros2_SDL2_audio_frontend/
├── docs/
│   ├── contributing/
│   │   ├── license.md
│   │   ├── rules.md
│   ├── usage/
│   │   ├── overview.md
│   │   ├── usage_with_linux.md
│   ├── index.md
├── sdl2_audio_frontend/
│   ├── config/
│   │   ├── audio_params.yaml
│   ├── include/
│   │   ├── sdl2_audio_frontend/
│   │   │   ├── audio/
│   │   │   │   ├── audio_async.hpp
│   │   │   ├── nodes/
│   │   │   │   ├── audio_capture_node.hpp
│   │   │   │   ├── audio_playback_node.hpp
│   │   │   │   ├── vad_node.hpp
│   │   │   │   ├── wake_word_node.hpp
│   ├── launch/
│   │   ├── audio_frontend.launch.py
│   ├── msg/
│   │   ├── AudioData.msg
│   ├── src/
│   │   ├── audio/
│   │   │   ├── audio_async.cpp
│   │   ├── nodes/
│   │   │   ├── audio_capture_node.cpp
│   │   │   ├── audio_playback_node.cpp
│   │   │   ├── vad_node.cpp
│   │   │   ├── wake_word_node.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
├── LICENSE
├── README.md
```

## Class diagram

```mermaid
classDiagram
    class AudioCaptureNode {
        +publish(audio_raw)
        -captureAudio()
    }
    
    class VADNode {
        +subscribe(audio_raw)
        +publish(voice_detected)
        -detectVoiceActivity()
    }
    
    class WakeWordNode {
        +subscribe(audio_raw)
        +subscribe(voice_detected)
        +publish(wake_word_detected)
        -detectWakeWord()
    }
    

    AudioCaptureNode ..> VADNode : audio_raw
    AudioCaptureNode ..> WakeWordNode : audio_raw
    
    VADNode ..> WakeWordNode : voice_detected
```

## Messages

```mermaid
classDiagram
    %% Message Types
    class AudioRawMsg {
        +header: Header
        +data: int16[]
        +sample_rate: uint32
        +channels: uint8
        +encoding: string
    }
    AudioCaptureNode ..> AudioRawMsg : publishes
```
```mermaid
classDiagram
    %% Message Types
    class VoiceDetectedMsg {
        +header: Header
        +is_voice: bool
        +start_time: Time
        +audio_data: int16[]
        +confidence: float32
    }
    %% Node connections to messages
    VADNode ..> VoiceDetectedMsg : publishes
```    
```mermaid
classDiagram
    %% Message Types
    class WakeWordMsg {
        +header: Header
        +is_wake_word: bool
        +wake_word: string
        +confidence: float32
        +start_timestamp: Time
        +end_timestamp: Time
    }
    %% Node connections to messages
    WakeWordNode ..> WakeWordMsg : publishes
``` 


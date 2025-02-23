# EdgeVox Project Documentation

# Project Structure
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
│   ├── project_knowledge.md
├── sdl2_audio_frontend/
│   ├── config/
│   │   ├── audio_params.yaml
│   │   ├── vad_params.yaml
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

# sdl2_audio_frontend/include/sdl2_audio_frontend/audio/audio_async.hpp
```cpp
#ifndef SDL2_AUDIO_FRONTEND_AUDIO_ASYNC_HPP
#define SDL2_AUDIO_FRONTEND_AUDIO_ASYNC_HPP

#include <SDL.h>
#include <SDL_audio.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace sdl2_audio_frontend {

class AudioAsync {
public:
    AudioAsync(int len_ms);
    ~AudioAsync();

    // Initialization
    bool init(int capture_id, int playback_id, int sample_rate);
    bool init(int capture_id, int sample_rate); // Backwards compatibility, capture only

    // Control
    bool resume();
    bool pause();
    bool clear();
    bool close();

    // Capture methods
    void get(int ms, std::vector<float>& audio);
    void capture_callback(uint8_t* stream, int len);

    // Playback methods
    bool play_audio(const std::vector<float>& audio);
    bool start_playback();
    bool stop_playback();
    bool is_playing() const;
    void playback_callback(uint8_t* stream, int len);
    void clear_playback();

private:
    SDL_AudioDeviceID capture_device_ = 0;
    SDL_AudioDeviceID playback_device_ = 0;
    int len_ms_ = 0;
    int sample_rate_ = 0;

    // State tracking
    std::atomic<bool> running_{false};
    std::atomic<bool> playing_{false};

    // Thread synchronization
    std::mutex mutex_;

    // Buffers
    std::vector<float> capture_buffer_;
    std::vector<float> playback_buffer_;
    size_t capture_buffer_pos_ = 0;
    size_t capture_buffer_len_ = 0;
};

// Return false if need to quit
bool sdl_poll_events();

} // namespace sdl2_audio_frontend

#endif // SDL2_AUDIO_FRONTEND_AUDIO_ASYNC_HPP
```


# sdl2_audio_frontend/include/sdl2_audio_frontend/nodes/audio_capture_node.hpp
```cpp
#ifndef SDL2_AUDIO_FRONTEND_AUDIO_CAPTURE_NODE_HPP
#define SDL2_AUDIO_FRONTEND_AUDIO_CAPTURE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sdl2_audio_frontend/msg/audio_data.hpp>
#include "sdl2_audio_frontend/audio/audio_async.hpp"

namespace sdl2_audio_frontend {

class AudioCaptureNode : public rclcpp::Node {
public:
    explicit AudioCaptureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AudioCaptureNode();

private:
    // Parameters
    void declare_parameters();
    void load_parameters();

    // Timer callback for audio capture
    void capture_timer_callback();

    // SDL2 audio interface
    std::unique_ptr<AudioAsync> audio_;
    
    // ROS2 publisher
    rclcpp::Publisher<msg::AudioData>::SharedPtr audio_pub_;
    
    // Timer for regular audio capture
    rclcpp::TimerBase::SharedPtr capture_timer_;

    // Parameters
    int sample_rate_;
    int capture_period_ms_;
    int buffer_length_ms_;
    int device_id_;
};

} // namespace sdl2_audio_frontend

#endif // SDL2_AUDIO_FRONTEND_AUDIO_CAPTURE_NODE_HPP
```


# sdl2_audio_frontend/include/sdl2_audio_frontend/nodes/audio_playback_node.hpp
```cpp


```


# sdl2_audio_frontend/include/sdl2_audio_frontend/nodes/vad_node.hpp
```cpp
#ifndef SDL2_AUDIO_FRONTEND_VAD_NODE_HPP
#define SDL2_AUDIO_FRONTEND_VAD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sdl2_audio_frontend/msg/audio_data.hpp>

namespace sdl2_audio_frontend {

class VADNode : public rclcpp::Node {
public:
    explicit VADNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VADNode() = default;

private:
    // Parameters
    void declare_parameters();
    void load_parameters();

    // Audio callback
    void audio_callback(const msg::AudioData::SharedPtr msg);

    // VAD implementation
    bool detect_voice_activity(const std::vector<float>& audio_data);
    
    // ROS subscribers and publishers
    rclcpp::Subscription<msg::AudioData>::SharedPtr audio_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr voice_detected_pub_;
    
    // VAD parameters
    float energy_threshold_;        // Energy threshold for VAD
    int min_samples_for_vad_;       // Minimum number of samples required for VAD
    float hold_time_sec_;           // How long to hold the VAD state after voice stops
    
    // State tracking
    bool current_vad_state_;        // Current VAD state
    rclcpp::Time last_voice_time_;  // Time when voice was last detected
};

} // namespace sdl2_audio_frontend

#endif // SDL2_AUDIO_FRONTEND_VAD_NODE_HPP
```


# sdl2_audio_frontend/include/sdl2_audio_frontend/nodes/wake_word_node.hpp
```cpp

```


# sdl2_audio_frontend/src/audio/audio_async.cpp
```cpp
#include "sdl2_audio_frontend/audio/audio_async.hpp"
#include <cstring> 

namespace sdl2_audio_frontend {

AudioAsync::AudioAsync(int len_ms) : len_ms_(len_ms) {}

AudioAsync::~AudioAsync() {
    if (capture_device_) {
        SDL_CloseAudioDevice(capture_device_);
    }
    if (playback_device_) {
        SDL_CloseAudioDevice(playback_device_);
    }
}

bool AudioAsync::init(int capture_id, int sample_rate) {
    return init(capture_id, -1, sample_rate);  // -1 means no playback device
}

bool AudioAsync::init(int capture_id, int playback_id, int sample_rate) {
    SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);

    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't initialize SDL: %s\n", SDL_GetError());
        return false;
    }

    SDL_SetHintWithPriority(SDL_HINT_AUDIO_RESAMPLING_MODE, "medium", SDL_HINT_OVERRIDE);

    // List available capture devices
    int nDevices = SDL_GetNumAudioDevices(SDL_TRUE);
    SDL_Log("Found %d capture devices:\n", nDevices);
    for (int i = 0; i < nDevices; i++) {
        SDL_Log("- Capture device #%d: '%s'\n", i, SDL_GetAudioDeviceName(i, SDL_TRUE));
    }

    // Configure capture spec
    SDL_AudioSpec capture_spec_requested, capture_spec_obtained;
    SDL_zero(capture_spec_requested);
    SDL_zero(capture_spec_obtained);

    capture_spec_requested.freq = sample_rate;
    capture_spec_requested.format = AUDIO_F32;
    capture_spec_requested.channels = 1;
    capture_spec_requested.samples = 1024;
    capture_spec_requested.callback = [](void* userdata, uint8_t* stream, int len) {
        auto* audio = static_cast<AudioAsync*>(userdata);
        audio->capture_callback(stream, len);
    };
    capture_spec_requested.userdata = this;

    // Open capture device
    const char* device_name = nullptr;  // Default device
    
    // If specific device requested, try to use that
    if (capture_id >= 0 && capture_id < SDL_GetNumAudioDevices(SDL_TRUE)) {
        device_name = SDL_GetAudioDeviceName(capture_id, SDL_TRUE);
        SDL_Log("Attempting to open specific capture device: %s\n", device_name);
    } else {
        SDL_Log("Attempting to open default capture device\n");
    }
    
    capture_device_ = SDL_OpenAudioDevice(device_name, SDL_TRUE, 
                                        &capture_spec_requested, &capture_spec_obtained, 0);
    
    if (!capture_device_) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't open capture device: %s\n", SDL_GetError());
        
        // If we failed with specified device, try with default
        if (device_name != nullptr) {
            SDL_Log("Trying to open default capture device instead\n");
            capture_device_ = SDL_OpenAudioDevice(nullptr, SDL_TRUE, 
                                                &capture_spec_requested, &capture_spec_obtained, 0);
        }
        
        // If still no capture device, check if we're opening playback
        if (!capture_device_ && playback_id < 0) {
            SDL_Log("No capture device opened and no playback requested\n");
            return false;
        }
    }

    // Configure playback spec
    SDL_AudioSpec playback_spec_requested, playback_spec_obtained;
    SDL_zero(playback_spec_requested);
    SDL_zero(playback_spec_obtained);

    playback_spec_requested.freq = sample_rate;
    playback_spec_requested.format = AUDIO_F32;
    playback_spec_requested.channels = 1;
    playback_spec_requested.samples = 1024;
    playback_spec_requested.callback = [](void* userdata, uint8_t* stream, int len) {
        auto* audio = static_cast<AudioAsync*>(userdata);
        audio->playback_callback(stream, len);
    };
    playback_spec_requested.userdata = this;

    // Open playback device if requested
    if (playback_id >= 0) {
        const char* play_device_name = playback_id > 0 ? 
            SDL_GetAudioDeviceName(playback_id, SDL_FALSE) : nullptr;
        
        playback_device_ = SDL_OpenAudioDevice(play_device_name, SDL_FALSE,
                                             &playback_spec_requested, &playback_spec_obtained, 0);
        
        if (!playback_device_) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Couldn't open playback device: %s\n", SDL_GetError());
            // Continue anyway, playback is optional
        }
    }

    // Update sample rate from the device we successfully opened
    if (capture_device_) {
        sample_rate_ = capture_spec_obtained.freq;
        SDL_Log("Opened capture device with sample rate: %d\n", sample_rate_);
    } else if (playback_device_) {
        sample_rate_ = playback_spec_obtained.freq;
        SDL_Log("No capture device, using playback sample rate: %d\n", sample_rate_);
    }

    // Resize buffers
    capture_buffer_.resize((sample_rate_ * len_ms_) / 1000);
    playback_buffer_.reserve(sample_rate_);  // Reserve 1 second

    return (capture_device_ != 0 || playback_device_ != 0);
}

bool AudioAsync::resume() {
    if (capture_device_) {
        SDL_PauseAudioDevice(capture_device_, 0);
    }
    if (playback_device_) {
        SDL_PauseAudioDevice(playback_device_, 0);
    }
    running_ = true;
    return true;
}

bool AudioAsync::pause() {
    if (capture_device_) {
        SDL_PauseAudioDevice(capture_device_, 1);
    }
    if (playback_device_) {
        SDL_PauseAudioDevice(playback_device_, 1);
    }
    running_ = false;
    playing_ = false;
    return true;
}

bool AudioAsync::clear() {
    if (!running_) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    capture_buffer_pos_ = 0;
    capture_buffer_len_ = 0;
    return true;
}

void AudioAsync::capture_callback(uint8_t* stream, int len) {
    if (!running_) return;

    size_t n_samples = len / sizeof(float);
    if (n_samples > capture_buffer_.size()) {
        n_samples = capture_buffer_.size();
        stream += (len - (n_samples * sizeof(float)));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (capture_buffer_pos_ + n_samples > capture_buffer_.size()) {
        const size_t n0 = capture_buffer_.size() - capture_buffer_pos_;
        std::memcpy(&capture_buffer_[capture_buffer_pos_], stream, n0 * sizeof(float));
        std::memcpy(&capture_buffer_[0], stream + n0 * sizeof(float), (n_samples - n0) * sizeof(float));
        capture_buffer_pos_ = (capture_buffer_pos_ + n_samples) % capture_buffer_.size();
        capture_buffer_len_ = capture_buffer_.size();
    } else {
        std::memcpy(&capture_buffer_[capture_buffer_pos_], stream, n_samples * sizeof(float));
        capture_buffer_pos_ = (capture_buffer_pos_ + n_samples) % capture_buffer_.size();
        capture_buffer_len_ = std::min(capture_buffer_len_ + n_samples, capture_buffer_.size());
    }
}

void AudioAsync::playback_callback(uint8_t* stream, int len) {
    if (!running_ || playback_buffer_.empty()) {
        std::memset(stream, 0, len);
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    size_t samples_needed = len / sizeof(float);
    size_t samples_to_copy = std::min(samples_needed, playback_buffer_.size());

    // Copy available samples
    std::memcpy(stream, playback_buffer_.data(), samples_to_copy * sizeof(float));

    // Fill remaining with silence if needed
    if (samples_to_copy < samples_needed) {
        std::memset(stream + (samples_to_copy * sizeof(float)), 0,
                   (samples_needed - samples_to_copy) * sizeof(float));
    }

    // Remove used samples
    playback_buffer_.erase(playback_buffer_.begin(), 
                          playback_buffer_.begin() + samples_to_copy);
}

void AudioAsync::get(int ms, std::vector<float>& result) {
    if (!capture_device_ || !running_) {
        result.clear();
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (ms <= 0) ms = len_ms_;

    size_t n_samples = (sample_rate_ * ms) / 1000;
    if (n_samples > capture_buffer_len_) {
        n_samples = capture_buffer_len_;
    }

    result.resize(n_samples);
    int64_t s0 = capture_buffer_pos_ - n_samples;
    if (s0 < 0) s0 += capture_buffer_.size();

    if (s0 + n_samples > capture_buffer_.size()) {
        const size_t n0 = capture_buffer_.size() - s0;
        std::memcpy(result.data(), &capture_buffer_[s0], n0 * sizeof(float));
        std::memcpy(&result[n0], &capture_buffer_[0], (n_samples - n0) * sizeof(float));
    } else {
        std::memcpy(result.data(), &capture_buffer_[s0], n_samples * sizeof(float));
    }
}

bool AudioAsync::play_audio(const std::vector<float>& audio) {
    if (!playback_device_) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    playback_buffer_.insert(playback_buffer_.end(), audio.begin(), audio.end());
    return true;
}

bool AudioAsync::start_playback() {
    if (!playback_device_) return false;
    SDL_PauseAudioDevice(playback_device_, 0);
    playing_ = true;
    return true;
}

bool AudioAsync::stop_playback() {
    if (!playback_device_) return false;
    SDL_PauseAudioDevice(playback_device_, 1);
    playing_ = false;
    return true;
}

bool AudioAsync::is_playing() const {
    return playback_device_ != 0 && playing_;
}

void AudioAsync::clear_playback() {
    std::lock_guard<std::mutex> lock(mutex_);
    playback_buffer_.clear();
}

bool AudioAsync::close() {
    if (capture_device_) {
        SDL_CloseAudioDevice(capture_device_);
        capture_device_ = 0;
    }
    if (playback_device_) {
        SDL_CloseAudioDevice(playback_device_);
        playback_device_ = 0;
    }

    SDL_QuitSubSystem(SDL_INIT_AUDIO);
    SDL_Quit();
    return true;
}

bool sdl_poll_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            return false;
        }
    }
    return true;
}

} // namespace sdl2_audio_frontend
```


# sdl2_audio_frontend/src/nodes/audio_playback_node.cpp
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("playback_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```


# sdl2_audio_frontend/src/nodes/vad_node.cpp
```cpp
#include "sdl2_audio_frontend/nodes/vad_node.hpp"

namespace sdl2_audio_frontend {

VADNode::VADNode(const rclcpp::NodeOptions& options)
    : Node("vad_node", options),
      current_vad_state_(false),
      last_voice_time_(this->now()) {
    
    // Declare and load parameters
    declare_parameters();
    load_parameters();
    
    // Create publisher for voice activity detection
    voice_detected_pub_ = create_publisher<std_msgs::msg::Bool>("voice_detected", 10);
    
    // Subscribe to audio data
    audio_sub_ = create_subscription<msg::AudioData>(
        "audio/raw", 
        10, 
        std::bind(&VADNode::audio_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "VAD node initialized");
    RCLCPP_INFO(get_logger(), "Energy threshold: %f", energy_threshold_);
    RCLCPP_INFO(get_logger(), "Min samples for VAD: %d", min_samples_for_vad_);
    RCLCPP_INFO(get_logger(), "Hold time: %f seconds", hold_time_sec_);
}

void VADNode::declare_parameters() {
    declare_parameter("energy_threshold", 0.01);   // Default energy threshold
    declare_parameter("min_samples_for_vad", 160); // Default 10ms @ 16kHz
    declare_parameter("hold_time_sec", 0.5);       // Default hold time in seconds
}

void VADNode::load_parameters() {
    energy_threshold_ = get_parameter("energy_threshold").as_double();
    min_samples_for_vad_ = get_parameter("min_samples_for_vad").as_int();
    hold_time_sec_ = get_parameter("hold_time_sec").as_double();
}

void VADNode::audio_callback(const msg::AudioData::SharedPtr msg) {
    // Convert raw audio bytes to float samples
    std::vector<float> audio_samples;
    audio_samples.resize(msg->samples);
    
    if (msg->format == 0) { // F32 format
        // Simply copy the data (it's already float)
        std::memcpy(audio_samples.data(), msg->data.data(), msg->data.size());
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "Unsupported audio format: %d", msg->format);
        return;
    }
    
    // Detect voice activity in the audio
    bool voice_active = detect_voice_activity(audio_samples);
    
    // Handle hold time logic
    if (voice_active) {
        last_voice_time_ = this->now();
        
        if (!current_vad_state_) {
            current_vad_state_ = true;
            RCLCPP_DEBUG(get_logger(), "Voice activity started");
        }
    } else {
        // Check if we're within the hold time
        rclcpp::Duration time_since_voice = this->now() - last_voice_time_;
        if (current_vad_state_ && time_since_voice.seconds() > hold_time_sec_) {
            current_vad_state_ = false;
            RCLCPP_DEBUG(get_logger(), "Voice activity ended");
        }
    }
    
    // Publish the current VAD state
    auto vad_msg = std::make_unique<std_msgs::msg::Bool>();
    vad_msg->data = current_vad_state_;
    voice_detected_pub_->publish(std::move(vad_msg));
}

bool VADNode::detect_voice_activity(const std::vector<float>& audio_data) {
    // Simple energy-based VAD
    if (audio_data.size() < static_cast<size_t>(min_samples_for_vad_)) {
        return false;
    }
    
    // Calculate energy of the audio
    float energy = 0.0f;
    for (float sample : audio_data) {
        energy += sample * sample;
    }
    energy /= audio_data.size();
    
    // Log energy level for debugging
    RCLCPP_DEBUG(get_logger(), "Audio energy: %f", energy);
    
    // Compare with threshold
    return energy > energy_threshold_;
}

} // namespace sdl2_audio_frontend

// Entry point
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sdl2_audio_frontend::VADNode>());
    rclcpp::shutdown();
    return 0;
}
```


# sdl2_audio_frontend/src/nodes/wake_word_node.cpp
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wake_word_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```


# sdl2_audio_frontend/src/nodes/audio_capture_node.cpp
```cpp
#include "sdl2_audio_frontend/nodes/audio_capture_node.hpp"

namespace sdl2_audio_frontend {

AudioCaptureNode::AudioCaptureNode(const rclcpp::NodeOptions& options)
    : Node("audio_capture_node", options) {

    // Declare and load parameters
    declare_parameters();
    load_parameters();

    // Log available audio devices
    RCLCPP_INFO(get_logger(), "Checking available audio devices...");
    audio_ = std::make_unique<AudioAsync>(buffer_length_ms_);
    
    if (!audio_->init(device_id_, sample_rate_)) {
        RCLCPP_WARN(get_logger(), "Failed to initialize with device_id %d, trying with 0", device_id_);
        
        // Try with first available device (index 0)
        if (!audio_->init(0, sample_rate_)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize audio capture");
            throw std::runtime_error("Audio initialization failed");
        }
    }

    // Create publisher
    audio_pub_ = create_publisher<msg::AudioData>("audio/raw", 10);

    // Create timer for regular audio capture
    auto capture_period = std::chrono::milliseconds(capture_period_ms_);
    capture_timer_ = create_wall_timer(
        capture_period,
        std::bind(&AudioCaptureNode::capture_timer_callback, this));

    // Start audio capture
    if (!audio_->resume()) {
        RCLCPP_ERROR(get_logger(), "Failed to start audio capture");
        throw std::runtime_error("Audio start failed");
    }

    RCLCPP_INFO(get_logger(), "Audio capture node initialized");
    RCLCPP_INFO(get_logger(), "Sample rate: %d Hz", sample_rate_);
    RCLCPP_INFO(get_logger(), "Capture period: %d ms", capture_period_ms_);
    RCLCPP_INFO(get_logger(), "Buffer length: %d ms", buffer_length_ms_);
}

AudioCaptureNode::~AudioCaptureNode() {
    if (audio_) {
        audio_->pause();
        audio_->close();
    }
}

void AudioCaptureNode::declare_parameters() {
    declare_parameter("sample_rate", 16000);
    declare_parameter("capture_period_ms", 10);
    declare_parameter("buffer_length_ms", 1000);
    declare_parameter("device_id", -1);  // -1 means default device
}

void AudioCaptureNode::load_parameters() {
    sample_rate_ = get_parameter("sample_rate").as_int();
    capture_period_ms_ = get_parameter("capture_period_ms").as_int();
    buffer_length_ms_ = get_parameter("buffer_length_ms").as_int();
    device_id_ = get_parameter("device_id").as_int();
}

void AudioCaptureNode::capture_timer_callback() {
    std::vector<float> audio_data;
    audio_->get(capture_period_ms_, audio_data);

    if (!audio_data.empty()) {
        auto msg = std::make_unique<msg::AudioData>();
        
        // Fill header
        msg->header.stamp = now();
        msg->header.frame_id = "audio_frame";
        
        // Fill audio metadata
        msg->sample_rate = sample_rate_;
        msg->format = 0;  // 0 for F32 format
        msg->channels = 1;  // Mono
        msg->samples = audio_data.size();
        
        // Convert float samples to bytes
        msg->data.resize(audio_data.size() * sizeof(float));
        std::memcpy(msg->data.data(), audio_data.data(), msg->data.size());

        audio_pub_->publish(std::move(msg));
    }
}

} // namespace sdl2_audio_frontend

// Entry point
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sdl2_audio_frontend::AudioCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
```


# sdl2_audio_frontend/CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(sdl2_audio_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add cmake modules directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  DEPENDENCIES std_msgs
)

# Need to link message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# SDL2 Audio utilities library
add_library(sdl2_audio_utils
  src/audio/audio_async.cpp
)
target_include_directories(sdl2_audio_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(sdl2_audio_utils PUBLIC
  ${SDL2_LIBRARIES}
)
target_compile_features(sdl2_audio_utils PUBLIC cxx_std_17)

# Audio Capture Node
add_executable(audio_capture_node src/nodes/audio_capture_node.cpp)
target_include_directories(audio_capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(audio_capture_node PUBLIC cxx_std_17)

target_link_libraries(audio_capture_node
  sdl2_audio_utils
  ${SDL2_LIBRARIES}  
)
target_link_libraries(audio_capture_node "${cpp_typesupport_target}")

ament_target_dependencies(audio_capture_node
  rclcpp
  std_msgs
)

# VAD Node
add_executable(vad_node src/nodes/vad_node.cpp)
target_include_directories(vad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(vad_node PUBLIC cxx_std_17)
target_link_libraries(vad_node
  sdl2_audio_utils
  "${cpp_typesupport_target}"
)
ament_target_dependencies(vad_node
  rclcpp
  std_msgs
)

# # Wake Word Node
# add_executable(wake_word_node src/nodes/wake_word_node.cpp)
# target_include_directories(wake_word_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(wake_word_node PUBLIC cxx_std_17)
# target_link_libraries(wake_word_node
#   sdl2_audio_utils
# )
# ament_target_dependencies(wake_word_node
#   rclcpp
#   std_msgs
# )

# Install library
install(TARGETS
  sdl2_audio_utils
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install targets
install(TARGETS
  audio_capture_node
  vad_node
  # wake_word_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```


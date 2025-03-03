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
│   ├── ros2_SDL2_audio_frontend_project_knowledge.md
├── sdl2_audio_frontend/
│   ├── config/
│   │   ├── audio_params.yaml
│   │   ├── vad_params.yaml
│   │   ├── wake_word_params.yaml
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
// wake_word_node.hpp
#ifndef SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP
#define SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sdl2_audio_frontend/msg/audio_data.hpp>
#include <memory>
#include <lowwi.hpp>

namespace sdl2_audio_frontend
{

    class WakeWordNode : public rclcpp::Node
    {
    public:
        explicit WakeWordNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~WakeWordNode();
        void configure_wake_word(); // Public method to be called after construction
        void test_with_sine_wave(); // Public method to test wake word detection with a sine wave

    private:
        // Parameter handling
        void declare_parameters();
        void load_parameters();

        // Callbacks
        void vad_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void audio_callback(const msg::AudioData::SharedPtr msg);
        void process_queued_audio();

        // Wake word detection callbacks
        static void wake_word_detected_static(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg);
        void wake_word_detected(const CLFML::LOWWI::Lowwi_ctx_t &ctx);

        // Lowwi instance
        std::unique_ptr<CLFML::LOWWI::Lowwi> ww_runtime_;

        // ROS communication
        rclcpp::Subscription<msg::AudioData>::SharedPtr audio_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vad_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wake_word_pub_;

        // Configuration
        std::string models_dir_;
        std::string model_filename_;
        std::string wake_word_phrase_;
        float confidence_threshold_;
        int min_activations_;
        int refractory_period_;

        // State tracking
        bool voice_active_ = false;
        std::vector<float> audio_buffer_;

        // Callback groups for parallel execution
        rclcpp::CallbackGroup::SharedPtr audio_callback_group_;
        rclcpp::CallbackGroup::SharedPtr processing_callback_group_;

        // Timer for processing audio in a separate thread
        rclcpp::TimerBase::SharedPtr process_timer_;

        // Audio queue and mutex for thread-safe access
        std::mutex audio_queue_mutex_;
        std::deque<std::vector<float>> audio_queue_;

        // Buffer settings
        size_t max_buffer_size_, batch_size_samples_;
    };

} // namespace sdl2_audio_frontend

#endif // SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP
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
    } else
        return false;

    // Resize buffers
    capture_buffer_.resize((sample_rate_ * len_ms_) / 1000);
    playback_buffer_.reserve(sample_rate_);  // Reserve 1 second

    return true;
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
#include "sdl2_audio_frontend/nodes/wake_word_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <filesystem>

namespace sdl2_audio_frontend
{

    WakeWordNode::WakeWordNode(const rclcpp::NodeOptions &options)
        : Node("wake_word_node", options)
    {
        audio_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        processing_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        declare_parameters();

        try
        {
            // Get the package share directory
            const std::string package_path = ament_index_cpp::get_package_share_directory("sdl2_audio_frontend");
            models_dir_ = package_path + "/models";
            RCLCPP_INFO(get_logger(), "Models directory: %s", models_dir_.c_str());
            std::string target_dir = "models"; // Relative to working directory

            // Create directory if it doesn't exist
            std::filesystem::create_directories(target_dir);

            // Copy all files from source to target
            for (const auto &entry : std::filesystem::recursive_directory_iterator(models_dir_))
            {
                if (entry.is_regular_file())
                {
                    std::filesystem::path target = target_dir / entry.path().lexically_relative(models_dir_);
                    std::filesystem::create_directories(target.parent_path());
                    std::filesystem::copy_file(entry.path(), target,
                                               std::filesystem::copy_options::overwrite_existing);
                }
            }

            // Initialize Lowwi runtime after setting the models path
            ww_runtime_ = std::make_unique<CLFML::LOWWI::Lowwi>();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
            throw; // Better to fail fast if we can't find our models
        }

        load_parameters();

        // Set up ROS publishers and subscribers
        wake_word_pub_ = create_publisher<std_msgs::msg::Bool>(
            "wake_word_detected", 10);

        // Create subscription options with callback groups
        rclcpp::SubscriptionOptions vad_options;
        vad_options.callback_group = audio_callback_group_;

        rclcpp::SubscriptionOptions audio_options;
        audio_options.callback_group = audio_callback_group_;

        // Set up subscribers with callback groups
        vad_sub_ = create_subscription<std_msgs::msg::Bool>(
            "voice_detected", 10,
            std::bind(&WakeWordNode::vad_callback, this, std::placeholders::_1),
            vad_options);

        audio_sub_ = create_subscription<msg::AudioData>(
            "audio/raw", 10,
            std::bind(&WakeWordNode::audio_callback, this, std::placeholders::_1),
            audio_options);
    }

    WakeWordNode::~WakeWordNode() = default;

    void WakeWordNode::configure_wake_word()
    {
        try
        {
            // Create full path to the model
            std::filesystem::path model_path = std::filesystem::path(models_dir_) / model_filename_;
            RCLCPP_INFO(get_logger(), "Using wake word model: %s", model_path.string().c_str());

            if (!std::filesystem::exists(model_path))
            {
                RCLCPP_ERROR(get_logger(), "Model file not found: %s", model_path.string().c_str());
                throw std::runtime_error("Wake word model file not found");
            }

            // Configure wake word detection
            CLFML::LOWWI::Lowwi_word_t ww;
            ww.phrase = wake_word_phrase_;
            ww.model_path = model_path;
            ww.threshold = confidence_threshold_;
            ww.min_activations = static_cast<float>(min_activations_);
            ww.refractory = refractory_period_;
            ww.cbfunc = &WakeWordNode::wake_word_detected_static;
            ww.cb_arg = std::static_pointer_cast<void>(shared_from_this());

            ww.debug = true; // Add this line to enable model-level debugging

            // Add to wakeword runtime
            ww_runtime_->add_wakeword(ww);

            RCLCPP_INFO(get_logger(), "Wake word configured with phrase: %s", wake_word_phrase_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to configure wake word: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Unknown exception during wake word configuration");
        }
    }

    void WakeWordNode::declare_parameters()
    {
        declare_parameter("model_filename", "example_wakewords/hey_mycroft.onnx");
        declare_parameter("wake_word_phrase", "Hey Mycroft");
        declare_parameter("confidence_threshold", 0.5);
        declare_parameter("min_activations", 5);
        declare_parameter("refractory_period", 20);
        declare_parameter("max_buffer_size_seconds", 5.0);
        declare_parameter("sample_rate", 16000);          // Must match the model's expected rate!
        declare_parameter("batch_size_ms", 1000);         // Process 1000ms of audio at once
        declare_parameter("processing_interval_ms", 100); // Process the queue every 100ms
    }

    void WakeWordNode::load_parameters()
    {
        // Get parameters
        get_parameter("model_filename", model_filename_);
        get_parameter("wake_word_phrase", wake_word_phrase_);
        get_parameter("confidence_threshold", confidence_threshold_);
        get_parameter("min_activations", min_activations_);
        get_parameter("refractory_period", refractory_period_);
        int batch_size_ms = get_parameter("batch_size_ms").as_int();
        int processing_interval_ms = get_parameter("processing_interval_ms").as_int();

        int sample_rate = get_parameter("sample_rate").as_int();
        if (sample_rate != 16000)
        {
            RCLCPP_WARN(get_logger(), "Sample rate %d Hz may not work with wake word model (expected 16000 Hz)", sample_rate);
        }
        batch_size_samples_ = static_cast<size_t>((sample_rate * batch_size_ms) / 1000);
        double buffer_seconds = get_parameter("max_buffer_size_seconds").as_double(); // Buffer should be at least 3x batch size for good overlap
        max_buffer_size_ = static_cast<size_t>(buffer_seconds * sample_rate);

        process_timer_ = create_wall_timer(
            std::chrono::milliseconds(processing_interval_ms),
            std::bind(&WakeWordNode::process_queued_audio, this),
            processing_callback_group_); // Use separate callback group for processing

        RCLCPP_INFO(get_logger(), "Wake word batch processing: %d ms batches, %d ms interval",
                    batch_size_ms, processing_interval_ms);
    }

    void WakeWordNode::vad_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool previous_state = voice_active_;
        voice_active_ = msg->data;

        // Add debug when state changes
        if (previous_state != voice_active_)
        {
            RCLCPP_DEBUG(get_logger(), "Voice activity state changed: %s",
                         voice_active_ ? "ACTIVE" : "INACTIVE");
        }

        if (!voice_active_ && previous_state)
        {
            // Voice activity ended, clear audio buffer
            size_t buffer_size = audio_buffer_.size();
            audio_buffer_.clear();
            RCLCPP_DEBUG(get_logger(), "Voice activity ended, cleared %zu samples from buffer", buffer_size);
        }
    }

    void WakeWordNode::audio_callback(const msg::AudioData::SharedPtr msg)
    {

        RCLCPP_DEBUG(get_logger(), "Received audio data: %d samples at %d Hz",
                     msg->samples, msg->sample_rate);

        // Convert raw audio bytes to float samples
        std::vector<float> audio_samples(msg->samples);

        if (msg->format == 0)
        { // F32 format
            // Simply copy the data (it's already float)
            std::memcpy(audio_samples.data(), msg->data.data(), msg->data.size());

            // Only queue data if voice is active
            if (voice_active_)
            {
                RCLCPP_DEBUG(get_logger(), "Voice active, queueing %zu audio samples", audio_samples.size());

                float energy = 0.0f;
                for (float sample : audio_samples)
                {
                    energy += sample * sample;
                }
                energy /= audio_samples.size();
                RCLCPP_DEBUG(get_logger(), "Audio energy: %f", energy);

                std::lock_guard<std::mutex> lock(audio_queue_mutex_);
                audio_queue_.push_back(std::move(audio_samples));

                RCLCPP_DEBUG(get_logger(), "Audio queue size: %zu batches", audio_queue_.size());
            }
            else
            {
                // Add debug when voice isn't active
                RCLCPP_DEBUG(get_logger(), "Voice not active, discarding audio samples");
            }
        }
        else
        {
            RCLCPP_WARN_ONCE(get_logger(), "Unsupported audio format: %d", msg->format);
        }
    }

    void WakeWordNode::process_queued_audio()
    {
        std::vector<std::vector<float>> batches_to_process;

        // Quickly grab all queued audio under lock
        {
            std::lock_guard<std::mutex> lock(audio_queue_mutex_);
            if (audio_queue_.empty())
            {
                return; // Nothing to process
            }

            RCLCPP_DEBUG(get_logger(), "Processing audio queue with %zu batches", audio_queue_.size());

            // Move all queued audio to our local vector
            while (!audio_queue_.empty())
            {
                batches_to_process.push_back(std::move(audio_queue_.front()));
                audio_queue_.pop_front();
            }
        }

        // Calculate total samples
        size_t total_samples = 0;
        for (const auto &batch : batches_to_process)
        {
            total_samples += batch.size();
        }
        RCLCPP_DEBUG(get_logger(), "Processing %zu audio batches with %zu total samples",
                     batches_to_process.size(), total_samples);

        // Add all batches to the circular buffer
        for (const auto &audio_data : batches_to_process)
        {
            size_t prev_size = audio_buffer_.size();
            audio_buffer_.insert(audio_buffer_.end(), audio_data.begin(), audio_data.end());

            RCLCPP_DEBUG(get_logger(), "Audio buffer size: %zu samples (added %zu)",
                         audio_buffer_.size(), audio_buffer_.size() - prev_size);
        }

        // Limit buffer size if needed
        if (audio_buffer_.size() > max_buffer_size_)
        {
            size_t samples_to_remove = audio_buffer_.size() - max_buffer_size_;
            RCLCPP_DEBUG(get_logger(), "Trimming buffer, removing %zu older samples", samples_to_remove);

            audio_buffer_.erase(audio_buffer_.begin(),
                                audio_buffer_.begin() + samples_to_remove);
        }

        // Only process if we have enough samples
        if (audio_buffer_.size() >= batch_size_samples_)
        {
            // Run on the entire batch (this is a big change from the original approach)
            std::vector<float> process_batch;

            // Take the most recent batch_size_samples_ from the buffer
            size_t start_idx = audio_buffer_.size() >= batch_size_samples_ ? audio_buffer_.size() - batch_size_samples_ : 0;

            process_batch.assign(
                audio_buffer_.begin() + start_idx,
                audio_buffer_.end());

            RCLCPP_DEBUG(get_logger(), "Running wake word detection on %zu samples batch",
                         process_batch.size());

            float min_val = 1.0f, max_val = -1.0f;
            for (float sample : process_batch)
            {
                min_val = std::min(min_val, sample);
                max_val = std::max(max_val, sample);
            }
            RCLCPP_INFO(get_logger(), "Audio range before detection: min=%f, max=%f", min_val, max_val);

            // Run the model on the combined larger batch
            // ww_runtime_->run(process_batch);

            // In process_queued_audio, before running the model
            std::vector<float> normalized_audio = process_batch;

            // Apply normalization similar to what the LOWWI demo does
            float max_value = 0.0f;
            for (float sample : normalized_audio)
            {
                max_value = std::max(max_value, std::abs(sample));
            }

            if (max_value > 0.0f)
            {
                // Scale to range [-0.95, 0.95] to avoid clipping
                float scale = 0.95f / max_value;
                for (float &sample : normalized_audio)
                {
                    sample *= scale;
                }
            }

            RCLCPP_DEBUG(get_logger(), "Running wake word detection on %zu normalized samples batch",
                         normalized_audio.size());

            // Run the model on the normalized audio
            ww_runtime_->run(normalized_audio);
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Not enough samples for a full batch (%zu < %zu)",
                         audio_buffer_.size(), batch_size_samples_);
        }
    }

    void WakeWordNode::wake_word_detected_static(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
    {
        // Convert shared_ptr back to node instance
        auto node_instance = std::static_pointer_cast<WakeWordNode>(arg);

        RCLCPP_DEBUG(node_instance->get_logger(),
                     "Wake word static callback triggered with phrase: '%s', confidence: %.2f%%",
                     ctx.phrase.c_str(), ctx.confidence);

        node_instance->wake_word_detected(ctx);
    }

    void WakeWordNode::wake_word_detected(const CLFML::LOWWI::Lowwi_ctx_t &ctx)
    {
        RCLCPP_INFO(get_logger(),
                    "Wake word detected: '%s' with confidence %.2f%%",
                    ctx.phrase.c_str(), ctx.confidence);

        // Add debug about buffer state
        RCLCPP_DEBUG(get_logger(), "Audio buffer at detection: %zu samples", audio_buffer_.size());

        // Publish wake word detection event
        auto msg = std::make_unique<std_msgs::msg::Bool>();
        msg->data = true;
        wake_word_pub_->publish(std::move(msg));
        RCLCPP_DEBUG(get_logger(), "Published wake word detection message");

        // Clear audio buffer after detection
        audio_buffer_.clear();
        RCLCPP_DEBUG(get_logger(), "Cleared audio buffer after detection");
    }

} // namespace sdl2_audio_frontend

// Entry point
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Use try-catch to handle exceptions during node creation
    try
    {
        auto node = std::make_shared<sdl2_audio_frontend::WakeWordNode>();

        node->configure_wake_word();

        // Use multithreaded executor with 2 threads
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("wake_word_node"), "Exception: %s", e.what());
        return 1;
    }

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
find_package(ament_index_cpp REQUIRED)
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

# Fetch Lightweight openwakeword implementation 
set (LOWWI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/Lowwi) 
link_directories(${LOWWI_PATH}/lib/x64) # Add the ONNX runtime library paths
link_directories(${LOWWI_PATH}/lib/x64/lib)
include(FetchContent)
FetchContent_Declare(
  Lowwi
  GIT_REPOSITORY https://github.com/CLFML/lowwi
  GIT_TAG main
  SOURCE_DIR ${LOWWI_PATH}
)
FetchContent_MakeAvailable(Lowwi)

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
  "${cpp_typesupport_target}"
)

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

# Wake Word Node
add_executable(wake_word_node src/nodes/wake_word_node.cpp)
message(STATUS "Lowwi source dir: ${Lowwi_SOURCE_DIR}")
target_include_directories(wake_word_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Lowwi_SOURCE_DIR}/src  # Lowwi headers
)

target_compile_features(wake_word_node PUBLIC cxx_std_17)
target_link_libraries(wake_word_node
  sdl2_audio_utils
  Lowwi  # Link with Lowwi library
  "${cpp_typesupport_target}"
  ncurses
)

ament_target_dependencies(wake_word_node
  rclcpp
  std_msgs
  ament_index_cpp
)

# Install libraries
install(TARGETS
  sdl2_audio_utils
  Lowwi
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY 
    "${Lowwi_SOURCE_DIR}/models"
    DESTINATION share/${PROJECT_NAME}
)

# # Custom command to copy models during build (for development)
# add_custom_target(copy_Lowwi_models ALL
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#         ${Lowwi_SOURCE_DIR}/models
#         ${CMAKE_CURRENT_BINARY_DIR}/models
#     COMMENT "Copying models directory to build folder"
# )

# Install ONNX Runtime from Lowwi's lib directory
install(FILES 
    "${LOWWI_PATH}/lib/x64/lib/libonnxruntime.so"
    "${LOWWI_PATH}/lib/x64/lib/libonnxruntime.so.1.18.0"
    DESTINATION lib
)

# Install targets
install(TARGETS
  audio_capture_node
  vad_node
  wake_word_node
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


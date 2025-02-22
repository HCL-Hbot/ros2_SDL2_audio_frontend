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
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
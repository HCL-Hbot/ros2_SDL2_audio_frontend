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
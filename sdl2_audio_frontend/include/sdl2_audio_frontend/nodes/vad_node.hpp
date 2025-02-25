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
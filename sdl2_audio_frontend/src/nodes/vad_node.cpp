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
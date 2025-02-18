#include "sdl2_audio_frontend/nodes/audio_capture_node.hpp"

namespace sdl2_audio_frontend {

AudioCaptureNode::AudioCaptureNode(const rclcpp::NodeOptions& options)
    : Node("audio_capture_node", options) {
    
    // Declare and load parameters
    declare_parameters();
    load_parameters();

    // Initialize SDL audio
    audio_ = std::make_unique<AudioAsync>(buffer_length_ms_);
    
    if (!audio_->init(device_id_, sample_rate_)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize audio capture");
        throw std::runtime_error("Audio initialization failed");
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
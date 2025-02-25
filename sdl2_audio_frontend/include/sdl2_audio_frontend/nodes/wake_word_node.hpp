// wake_word_node.hpp
#ifndef SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP
#define SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sdl2_audio_frontend/msg/audio_data.hpp>
#include <memory>
#include <lowwi.hpp>

namespace sdl2_audio_frontend {

class WakeWordNode : public rclcpp::Node {  // Remove std::enable_shared_from_this
public:
    explicit WakeWordNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~WakeWordNode();

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
    void wake_word_detected(const CLFML::LOWWI::Lowwi_ctx_t& ctx);

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
    size_t max_buffer_size_ = 16000 * 5;  // 5 seconds at 16kHz by default        
};

} // namespace sdl2_audio_frontend

#endif // SDL2_AUDIO_FRONTEND_WAKE_WORD_NODE_HPP
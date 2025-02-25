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

            // Add to wakeword runtime  
            ww_runtime_->add_wakeword(ww);

            RCLCPP_INFO(get_logger(), "Wake word configured with phrase: %s", wake_word_phrase_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to configure wake word: %s", e.what());
        } catch (...) {
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
        declare_parameter("sample_rate", 16000); // Must match the model's expected rate
    }

    void WakeWordNode::load_parameters()
    {
        // Get parameters
        get_parameter("model_filename", model_filename_);
        get_parameter("wake_word_phrase", wake_word_phrase_);
        get_parameter("confidence_threshold", confidence_threshold_);
        get_parameter("min_activations", min_activations_);
        get_parameter("refractory_period", refractory_period_);

        int sample_rate = get_parameter("sample_rate").as_int();
        if (sample_rate != 16000)
        {
            RCLCPP_WARN(get_logger(), "Sample rate %d Hz may not work with wake word model (expected 16000 Hz)", sample_rate);
        }
        double buffer_seconds = get_parameter("max_buffer_size_seconds").as_double();
        max_buffer_size_ = static_cast<size_t>(buffer_seconds * 16000.0); // Assuming 16kHz sample rate
    }

    void WakeWordNode::vad_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        voice_active_ = msg->data;

        if (!voice_active_)
        {
            // Voice activity ended, clear audio buffer
            audio_buffer_.clear();
        }
    }

    void WakeWordNode::audio_callback(const msg::AudioData::SharedPtr msg)
    {
        // Convert raw audio bytes to float samples
        std::vector<float> audio_samples(msg->samples);

        if (msg->format == 0)
        { // F32 format
            // Simply copy the data (it's already float)
            std::memcpy(audio_samples.data(), msg->data.data(), msg->data.size());

            // Only queue data if voice is active
            if (voice_active_)
            {
                std::lock_guard<std::mutex> lock(audio_queue_mutex_);
                audio_queue_.push_back(std::move(audio_samples));
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

            // Move all queued audio to our local vector
            while (!audio_queue_.empty())
            {
                batches_to_process.push_back(std::move(audio_queue_.front()));
                audio_queue_.pop_front();
            }
        }

        // Process each batch (potentially time-consuming)
        for (const auto &audio_data : batches_to_process)
        {
            // Add to continuous buffer if needed
            audio_buffer_.insert(audio_buffer_.end(), audio_data.begin(), audio_data.end());

            // Limit buffer size if needed
            if (audio_buffer_.size() > max_buffer_size_)
            {
                audio_buffer_.erase(audio_buffer_.begin(),
                                    audio_buffer_.begin() + (audio_buffer_.size() - max_buffer_size_));
            }

            // Run wake word detection (potentially expensive operation)
            ww_runtime_->run(audio_data);
        }
    }

    void WakeWordNode::wake_word_detected_static(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
    {
        // Convert shared_ptr back to node instance
        auto node_instance = std::static_pointer_cast<WakeWordNode>(arg);
        node_instance->wake_word_detected(ctx);
    }

    void WakeWordNode::wake_word_detected(const CLFML::LOWWI::Lowwi_ctx_t &ctx)
    {
        RCLCPP_INFO(get_logger(),
                    "Wake word detected: '%s' with confidence %.2f%%",
                    ctx.phrase.c_str(), ctx.confidence);

        // Publish wake word detection event
        auto msg = std::make_unique<std_msgs::msg::Bool>();
        msg->data = true;
        wake_word_pub_->publish(std::move(msg));

        // Clear audio buffer after detection
        audio_buffer_.clear();
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
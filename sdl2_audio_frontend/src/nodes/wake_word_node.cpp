#include "sdl2_audio_frontend/nodes/wake_word_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <filesystem>
#include <fstream>
#include <vector>
#include <iostream>


/**
 * @brief Callback that's called when wakeword is triggered 
 */
void wakeword_callback(CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void> arg)
{
    std::cout << "Hey! who said \"" << ctx.phrase << "\"?" << ", I am for " << ctx.confidence << " % certain, I heard something!\n"
              << std::endl;
}

/**
 * @brief Loads in .wav files and stores them in float vector
 */
std::vector<float> load_audio_samples(const std::string &file_path)
{
    // Open the WAV file in binary mode
    std::ifstream file(file_path, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    // Read the WAV header (44 bytes)
    char header[44];
    file.read(header, 44);
    if (file.gcount() < 44)
    {
        throw std::runtime_error("Invalid WAV file: Header is too short");
    }

    // Validate file format
    if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F' ||
        header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E')
    {
        throw std::runtime_error("Invalid WAV file: Incorrect RIFF or WAVE header");
    }

    // Read audio data as 16-bit signed integers
    std::vector<float> samples;
    while (!file.eof())
    {
        int16_t sample;
        file.read(reinterpret_cast<char *>(&sample), sizeof(int16_t));
        if (file.gcount() == sizeof(int16_t))
        {
            samples.push_back(static_cast<float>(sample));
        }
    }

    return samples;
}


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

            RCLCPP_INFO(get_logger(), "Wake word configured with phrase: %s", ww.phrase.c_str());
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

    void WakeWordNode::test_with_wav_file(std::string wav_path)
    {
        RCLCPP_INFO(get_logger(), "Testing with WAV file: %s", wav_path.c_str());

        // // Read WAV file
        // std::ifstream file(wav_path, std::ios::binary);
        // if (!file.is_open())
        // {
        //     RCLCPP_ERROR(get_logger(), "Failed to open WAV file");
        //     return;
        // }

        // // Skip WAV header (44 bytes)
        // char header[44];
        // file.read(header, 44);

        // // Check if it's a valid WAV file
        // if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F' ||
        //     header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E')
        // {
        //     RCLCPP_ERROR(get_logger(), "Invalid WAV file");
        //     return;
        // }

        // // Read audio data as 16-bit signed integers and convert to float
        // std::vector<float> audio_samples;
        // int16_t sample;
        // while (file.read(reinterpret_cast<char *>(&sample), sizeof(sample)))
        // {
        //     // Convert 16-bit PCM to float in range [-1.0, 1.0]
        //     audio_samples.push_back(static_cast<float>(sample) / 32768.0f);
        // }

        std::vector<float> audio_samples = load_audio_samples(wav_path);
        RCLCPP_INFO(get_logger(), "Loaded %zu samples from WAV file", audio_samples.size());

        // Analyze the audio to confirm it looks good
        float min_val = *std::min_element(audio_samples.begin(), audio_samples.end());
        float max_val = *std::max_element(audio_samples.begin(), audio_samples.end());
        RCLCPP_INFO(get_logger(), "WAV file audio range: min=%f, max=%f", min_val, max_val);

        // Create a new Lowwi runtime with the exact same config as your standalone demo
        auto test_runtime = std::make_unique<CLFML::LOWWI::Lowwi>();
        // CLFML::LOWWI::Lowwi test_runtime;

        // Configure wake word with the same parameters as the standalone demo
        CLFML::LOWWI::Lowwi_word_t test_ww;
        test_ww.phrase = "Hey Mycroft";
        test_ww.model_path = std::filesystem::path("models/example_wakewords/hey_mycroft.onnx"); // Use the path expected by Lowwi
        test_ww.cbfunc = wakeword_callback;

        // test_ww.threshold = 0.5f;                                         // Use exact values from demo_mic.cpp
        // test_ww.min_activations = 5.0f;
        // test_ww.refractory = 20;
        // test_ww.debug = true; // Enable debug mode

        // // Use a simple lambda for callback
        // test_ww.cbfunc = [this](CLFML::LOWWI::Lowwi_ctx_t ctx, std::shared_ptr<void>)
        // {
        //     RCLCPP_INFO(this->get_logger(),
        //                 "TEST WAV: Wake word detected: '%s' with confidence %.2f%%",
        //                 ctx.phrase.c_str(), ctx.confidence);
        // };

        // Add the wake word to the runtime
        test_runtime->add_wakeword(test_ww);

        // Run detection on the test audio
        RCLCPP_INFO(get_logger(), "Running wake word detection on test data; should activate!");
        test_runtime->run(audio_samples);

        // std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        // audio_queue_.push_back(std::move(audio_samples));

        RCLCPP_INFO(get_logger(), "WAV file test complete");
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

    void analyze_audio(const std::vector<float> &audio, const std::string &source)
    {
        if (audio.empty())
            return;

        float min_val = *std::min_element(audio.begin(), audio.end());
        float max_val = *std::max_element(audio.begin(), audio.end());

        double sum = 0.0, sum_sq = 0.0;
        int zero_crossings = 0;
        float prev_sample = 0.0f;

        std::vector<int> histogram(10, 0); // Simple amplitude histogram

        for (size_t i = 0; i < audio.size(); i++)
        {
            float sample = audio[i];
            sum += sample;
            sum_sq += sample * sample;

            // Count zero crossings
            if (i > 0 && ((prev_sample >= 0 && sample < 0) || (prev_sample < 0 && sample >= 0)))
            {
                zero_crossings++;
            }
            prev_sample = sample;

            // Fill histogram
            int bin = std::min(9, static_cast<int>(std::abs(sample) * 10.0f));
            histogram[bin]++;
        }

        float mean = sum / audio.size();
        float rms = std::sqrt(sum_sq / audio.size());
        float zero_crossing_rate = static_cast<float>(zero_crossings) / audio.size();

        std::cout << source << " audio analysis:" << std::endl
                  << "  Samples: " << audio.size() << std::endl
                  << "  Range: " << min_val << " to " << max_val << std::endl
                  << "  Mean: " << mean << ", RMS: " << rms << std::endl
                  << "  Zero crossing rate: " << zero_crossing_rate << std::endl
                  << "  Histogram: ";

        for (int count : histogram)
        {
            std::cout << count << " ";
        }
        std::cout << std::endl;
    }

    void WakeWordNode::process_queued_audio()
    {
        std::vector<std::vector<float>> batches_to_process;

        // std::cout << "Processing queued audio" << std::endl;    

        // Quickly grab all queued audio under lock
        {
            std::lock_guard<std::mutex> lock(audio_queue_mutex_);
            // std::cout << "Audio queue size: " << audio_queue_.size() << " batches" << std::endl;
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

            analyze_audio(process_batch, "ROS2 Node");

            // Run the model on the combined larger batch
            ww_runtime_->run(process_batch);
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

    try
    {
        auto node = std::make_shared<sdl2_audio_frontend::WakeWordNode>();

        // node->configure_wake_word();

        node->test_with_wav_file("/home/jeroen/local_dev/ros2_SDL2_audio_frontend/sdl2_audio_frontend/lib/Lowwi/example/LOWWI_demo_fragment/example_fragments/hey_mycroft_test.wav");

        // // Use multithreaded executor with 2 threads
        // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        // executor.add_node(node);
        // executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("wake_word_node"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
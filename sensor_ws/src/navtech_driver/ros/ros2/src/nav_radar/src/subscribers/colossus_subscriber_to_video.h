#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>


class Colossus_subscriber_to_video : public ::rclcpp::Node {
public:
    Colossus_subscriber_to_video();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    bool config_data_received{ false };
    cv::VideoWriter video_writer{};
    int encoder_size{ 0 };
    int azimuth_samples{ 0 };
    int video_width{ 0 };
    int video_height{ 0 };
    int current_bearing{ 0 };
    int azimuth{ 0 };
    int data_length{ 0 };
    int expected_rotation_rate{ 0 };
    std::string output_folder_name {"output_videos"};
    cv::Mat radar_image{ cv::Size{400, 400}, CV_8UC1, cv::Scalar{0, 0} };
    cv::Mat blank_image{ cv::Size{400, 400}, CV_8UC1, cv::Scalar{0, 0} };
    uint8_t* image_ptr = { (uint8_t*)radar_image.data };
    int last_azimuth{ 0 };

    void configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const;
    void fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg) const;

    rclcpp::Subscription<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<navtech_msgs::msg::RadarFftDataMsg>::SharedPtr fft_data_subscriber;
};

extern std::shared_ptr<Colossus_subscriber_to_video> node;
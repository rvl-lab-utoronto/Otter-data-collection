#include <rclcpp/rclcpp.hpp>


class Colossus_subscriber : public ::rclcpp::Node
{
public:
    Colossus_subscriber();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    void configuration_data_callback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) const;
    void fft_data_callback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg) const;

    rclcpp::Subscription<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<navtech_msgs::msg::RadarFftDataMsg>::SharedPtr fft_data_subscriber;
};
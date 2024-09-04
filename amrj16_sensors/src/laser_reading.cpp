#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarReader : public rclcpp::Node
{
public:
    LidarReader()
    : Node("lidar_reader")
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/amrj16/laser_scan", qos_profile,
            std::bind(&LidarReader::lidar_callback, this, std::placeholders::_1));
    }


private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Define the angle of interest in radians
        double angle_of_interest = 1.5707;  // Example angle of interest (in radians)
        
        // Find the index corresponding to the angle of interest
        double angle_increment = msg->angle_increment;
        int index_of_interest = static_cast<int>(angle_of_interest / angle_increment);

        // Get the range and intensity values at the angle of interest
        float range_at_angle = msg->ranges[index_of_interest];
        float intensity_at_angle = msg->intensities[index_of_interest];

        // Find the maximum intensity measured in the scan
        float max_intensity = *std::max_element(msg->intensities.begin(), msg->intensities.end());

        // Print the range and intensity at the angle of interest
        RCLCPP_INFO(this->get_logger(), "Range at %f radians: %f", angle_of_interest, range_at_angle);
        RCLCPP_INFO(this->get_logger(), "Intensity at %f radians: %f", angle_of_interest, intensity_at_angle);

        // Print the maximum intensity measured in the scan
        RCLCPP_INFO(this->get_logger(), "Maximum intensity measured: %f", max_intensity);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto lidar_reader = std::make_shared<LidarReader>();
    rclcpp::spin(lidar_reader);
    rclcpp::shutdown();
    return 0;
}
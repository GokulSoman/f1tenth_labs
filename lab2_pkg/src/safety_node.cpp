#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class TemplateNode : public rclcpp::Node {
    public:
        TemplateNode() : Node("safety_node") {

        }
    private:
        
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateNode>());
    rclcpp::shutdown();
    return 0;
}

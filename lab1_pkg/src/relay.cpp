#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class RelayNode : public rclcpp::Node {

    public:
        RelayNode() : Node("relay") {


            publisher_ = this-> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
            subsriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10,
                                std::bind(&RelayNode::subscriber_callback, this, std::placeholders::_1));
            
            
            RCLCPP_INFO(this->get_logger(), "Relay node is initialized.");
        }
    private:
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subsriber_;

        void subscriber_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
            ackermann_msgs::msg::AckermannDriveStamped new_msg;
            new_msg.drive.speed = msg->drive.speed * 3;
            new_msg.drive.steering_angle = msg->drive.steering_angle * 3;
            publisher_->publish(new_msg);
            RCLCPP_INFO(this->get_logger(),"New speed: %.3f, New steering angle: %.3f" ,
                                        new_msg.drive.speed,
                                        new_msg.drive.steering_angle);

        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayNode>());
    rclcpp::shutdown();
    return 0;
}
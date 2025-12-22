#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

class TalkerNode : public rclcpp::Node{
    public:
        TalkerNode() : Node("talker_node")
        {
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("ackermann_steering", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                        std::bind(&TalkerNode::callback_publisher, this));
            RCLCPP_INFO(this->get_logger(), "Initialized tha ackermann publisher");
        }

    private:

        // publisher_-> publish(); 

        rclcpp::TimerBase::SharedPtr timer_ ;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
        ackermann_msgs::msg::AckermannDrive msg;
        void callback_publisher(){
            msg.acceleration = 1;
            msg.speed = 1;
            msg.jerk = 1;
            publisher_->publish(msg);
        }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}
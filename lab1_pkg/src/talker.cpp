#include <rclcpp/rclcpp.hpp>
// #include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class TalkerNode : public rclcpp::Node{
    public:
        TalkerNode() : Node("talker_node")
        {   

            this->declare_parameter<double>("v", 0.0);
            this->declare_parameter<double>("d", 0.0);


            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_steering", 10);
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
            //                             std::bind(&TalkerNode::callback_publisher, this));

            // publish on parameter set
            param_handler = this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> &parameters){

                    rcl_interfaces::msg::SetParametersResult result;


                    // reset result
                    result.successful = false;
                    result.reason = "";


                    for (auto &p : parameters){
                        if (p.get_name() == "v") {
                            speed = p.as_double();
                            result.successful = true;
                            result.reason = "Parameter v: set as " + std::to_string(speed) +  ", ";

                        }
                        if (p.get_name() == "d") {
                            steering_angle = p.as_double();
                            result.successful = true;
                            result.reason = "Parameter d: set as " + std::to_string(steering_angle) + ", " ;

                        }
                    }
                
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), result.reason);
                    this -> callback_publisher(speed, steering_angle);
                }
                return result;

                }
            );

            RCLCPP_INFO(this->get_logger(), "Initialized tha ackermann publisher");

        }


    private:

        // publisher_-> publish(); 

        rclcpp::TimerBase::SharedPtr timer_ ;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler;
        ackermann_msgs::msg::AckermannDriveStamped msg;
        
        void callback_publisher(double v, double d){
            auto msg = ackermann_msgs::msg::AckermannDriveStamped();
            msg.header.stamp = this->now();
            msg.drive.speed = v;
            msg.drive.steering_angle = d;
            // msg.drive.acceleration = 1;
            // msg.drive.speed = 1;
            // msg.drive.jerk = 1;
            publisher_->publish(msg);
        }
        double speed{0.0}, steering_angle{0.0};
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}
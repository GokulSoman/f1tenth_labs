#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>
#include <limits>

class TemplateNode : public rclcpp::Node {
    public:
        TemplateNode() : Node("safety_node") {

            scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
                                std::bind(&TemplateNode::scan_sub_callback, this, std::placeholders::_1));
            odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10,
                                std::bind(&TemplateNode::odom_sub_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Safety node initialized.");
        }
    private:
        
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher;
        Eigen::ArrayXf prev_ranges, range_diff, vel, ttc, rads;
        float time_prev = 0, dt=0;

        void scan_sub_callback(const sensor_msgs::msg::LaserScan msg){
            
            Eigen::Map<const Eigen::ArrayXf> curr_range(
                msg.ranges.data(), msg.ranges.size()
            );
            double curr_time = static_cast<double>(msg.header.stamp.sec)
                            + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
            if (prev_ranges.size() == 0){
                // intializing steps
                prev_ranges = curr_range;
                time_prev = curr_time;

                //find rad vals
                double angle_min = msg.angle_min ;
                double angle_max = msg.angle_max;
                double angle_incr = msg.angle_increment;
                int num_rads = static_cast<int> (std::floor((angle_max - angle_min)/angle_incr));

                if (num_rads != prev_ranges.size()){
                    RCLCPP_ERROR(this->get_logger(), "Number of radians {%d} not matching lidar range size {%d}",
                                num_rads, prev_ranges.size());
                }
                
                rads = Eigen::ArrayXf::LinSpaced(num_rads, angle_min, angle_max);
                

            } 
            else {

                if (prev_ranges.size() != curr_range.size()){
                    RCLCPP_WARN(this->get_logger(), "The range lengths are not matching, prev %d, curr %d",
                                            prev_ranges.size(), curr_range.size());
                }

                range_diff = prev_ranges - curr_range;
                dt = curr_time - time_prev;
                
                vel = (range_diff / dt); // .min(0.0);
                vel = (vel >= 0.0f).select(std::numeric_limits<float>::infinity(), vel);
                ttc = curr_range / -vel;

                // check whether longitudinal velocity matches low ttc 

                // set new vals
                prev_ranges = curr_range;
                time_prev = curr_time;
            }

        }

        void odom_sub_callback(nav_msgs::msg::Odometry odom){

        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateNode>());
    rclcpp::shutdown();
    return 0;
}

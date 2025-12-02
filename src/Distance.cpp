#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#include <string.h>
#include <math.h>

using std::placeholders::_1;

class DistanceNode : public rclcpp::Node
{
    public:
        DistanceNode() : Node("distance_node")
        {
            publisher1_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10); // To publish ditances between turtle1 and turtle2
            publisher2_ = this->create_publisher<std_msgs::msg::Float32>("boundary_condition", 10); // To publish eventual violation of boundaries
            subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,std::bind(&DistanceNode::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,std::bind(&DistanceNode::topic_callback2, this, _1));
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback1, this));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback2, this));
        }
    private:
        void topic_callback1(const turtlesim::msg::Pose::SharedPtr pos){
            pos_turtle1_ = *pos;
        }
        void topic_callback2(const turtlesim::msg::Pose::SharedPtr pos){
            pos_turtle2_ = *pos;
        }
        void timer_callback1(){
            distance_.data = sqrt(pow(pos_turtle1_.x - pos_turtle2_.x,2)+pow(pos_turtle1_.y - pos_turtle2_.y,2));
            if(pos_turtle1_.x < pos_turtle2_.x) // Turtle 2 is in front of turtle 1
                distance_.data = distance_.data*-1.0;
            RCLCPP_INFO(this->get_logger(), "Distance: %f",distance_.data);
            publisher1_->publish(distance_);
        }
        void timer_callback2(){
            boundary_condition_.data = 0.0;
            if(pos_turtle1_.x >= 10.0 || pos_turtle1_.y >= 10.0 || 
                pos_turtle1_.x <= 2.0 || pos_turtle1_.y <= 2.0){
                if(pos_turtle1_.x >= 10.0)
                // Boundary reached on x
                    boundary_condition_.data = 1.1;
                if(pos_turtle1_.x <= 2.0)
                    // Boundary reached on x
                    boundary_condition_.data = -1.1;
                if(pos_turtle1_.y >= 10.0)
                    // Boundary reached on y
                    boundary_condition_.data = boundary_condition_.data + 0.1;
                if(pos_turtle1_.y <= 2.0)
                    // Boundary reached on y
                    boundary_condition_.data = boundary_condition_.data - 0.1;
                }
            else if(pos_turtle2_.x >= 10.0 || pos_turtle2_.y >= 10.0 || 
                    pos_turtle2_.x <= 2.0 || pos_turtle2_.y <= 2.0){
                if(pos_turtle2_.x >= 10.0)
                    // Boundary reached on x
                    boundary_condition_.data = 2.1;
                if(pos_turtle2_.x <= 2.0)
                    // Boundary reached on x
                    boundary_condition_.data = -2.1;
                if(pos_turtle2_.y >= 10.0)
                    // Boundary reached on y
                    boundary_condition_.data = boundary_condition_.data + 0.2;
                if(pos_turtle2_.y <= 2.0)
                    // Boundary reached on y
                    boundary_condition_.data = boundary_condition_.data - 0.2;
                }
            else
                boundary_condition_.data = 0.0;
            publisher2_->publish(boundary_condition_);
        }
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher1_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher2_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription2_;
        std_msgs::msg::Float32 distance_;
        std_msgs::msg::Float32 boundary_condition_;
        turtlesim::msg::Pose pos_turtle1_;
        turtlesim::msg::Pose pos_turtle2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
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
            publisher1_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10); // To publish distances between turtle1 and turtle2
            publisher2_ = this->create_publisher<std_msgs::msg::Float32>("boundary_condition_on_1", 10); // To publish eventual violation of boundaries
            publisher3_ = this->create_publisher<std_msgs::msg::Float32>("boundary_condition_on_2", 10); // To publish eventual violation of boundaries
            publisher4_ = this->create_publisher<std_msgs::msg::Float32>("relative_pos", 10); // To publish the relative placement of the two turtles
            subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,std::bind(&DistanceNode::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,std::bind(&DistanceNode::topic_callback2, this, _1));
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback1, this));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback2, this));
            timer3_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback3, this));
            timer4_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::timer_callback4, this));
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
            publisher1_->publish(distance_);
        }
        void timer_callback2(){
            boundary_condition_on_1_.data = 0.0;
            if(pos_turtle1_.x >= 10.0){
                // Right boundary reached
                if((pos_turtle1_.theta < 1.57 && pos_turtle1_.theta >= 0.0) || (pos_turtle1_.theta > -1.57 && pos_turtle1_.theta <= 0.0))
                    // Facing boundary
                    boundary_condition_on_1_.data = 1.0;
                else
                    boundary_condition_on_1_.data = 1.5;
            }
            if(pos_turtle1_.x <= 1.0){
                // Left boundary reached
                if((pos_turtle1_.theta > 1.57 && pos_turtle1_.theta <= 3.14) || (pos_turtle1_.theta >= -3.14 && pos_turtle1_.theta < -1.57))
                    // Facing boundary
                    boundary_condition_on_1_.data = 2.0;
                else
                    boundary_condition_on_1_.data = 2.5;
            }
            if(pos_turtle1_.y >= 10.0){
                // Upper boundary reached
                if(pos_turtle1_.theta > 0.0 && pos_turtle1_.theta < 3.14)
                    // Facing boundary
                    boundary_condition_on_1_.data = 3.0;
                else
                    boundary_condition_on_1_.data = 3.5;
            }
            if(pos_turtle1_.y <= 1.0){
                // Boundary reached on y
                if(pos_turtle1_.theta < 0.0 && pos_turtle1_.theta > -3.14)
                    // Facing boundary
                    boundary_condition_on_1_.data = 4.0;
                else
                    boundary_condition_on_1_.data = 4.5;
            }
            publisher2_->publish(boundary_condition_on_1_);
        }
        void timer_callback3(){
            boundary_condition_on_2_.data = 0.0;
            if(pos_turtle2_.x >= 10.0){
                // Right boundary reached
                if((pos_turtle2_.theta < 1.57 && pos_turtle2_.theta >= 0.0) || (pos_turtle2_.theta > -1.57 && pos_turtle2_.theta <= 0.0))
                    // Facing boundary
                    boundary_condition_on_2_.data = 1.0;
                else
                    boundary_condition_on_2_.data = 1.5;
            }
            if(pos_turtle2_.x <= 1.0){
                // Left boundary reached
                if((pos_turtle2_.theta > 1.57 && pos_turtle2_.theta <= 3.14) || (pos_turtle2_.theta >= -3.14 && pos_turtle2_.theta < -1.57))
                    // Facing boundary
                    boundary_condition_on_2_.data = 2.0;
                else
                    boundary_condition_on_2_.data = 2.5;
            }
            if(pos_turtle2_.y >= 10.0){
                // Upper boundary reached   
                if(pos_turtle2_.theta > 0.0 && pos_turtle2_.theta < 3.14)
                    // Facing boundary
                    boundary_condition_on_2_.data = 3.0;
                else
                    boundary_condition_on_2_.data = 3.5;
            }
            if(pos_turtle2_.y <= 1.0){
                // Lower boundary reached
                if(pos_turtle2_.theta < 0.0 && pos_turtle2_.theta > -3.14)
                    // Facing boundary
                    boundary_condition_on_2_.data = 4.0;
                else
                    boundary_condition_on_2_.data = 4.5;
            }
            publisher3_->publish(boundary_condition_on_2_);
        }
        void timer_callback4(){
            relative_pos_.data = 0.0;
            // Classifying each configuration of turtle1 and turtle2
            if(pos_turtle1_.x > pos_turtle2_.x){
                if(pos_turtle1_.y >= pos_turtle2_.y){
                    // Turtle1 is at the top right relative to turtle2
                    if(pos_turtle1_.theta <= -3.14 && pos_turtle1_.theta >= -1.57 &&
                        pos_turtle2_.theta >= 0.0 && pos_turtle2_.theta <= 1.57)
                        // Two turtles look at each other
                        relative_pos_.data = 1.0;
                    else if(pos_turtle1_.theta <= -3.14 && pos_turtle1_.theta >= -1.57)
                        // Turtle1 looks towards turtle2
                        relative_pos_.data = 1.1;
                    else if(pos_turtle2_.theta >= 0.0 && pos_turtle2_.theta <= 1.57)
                        // Turtle2 looks towards turtle1
                        relative_pos_.data = 1.2;
                }
                else{
                    // Turtle1 is at the bottom right relative to turtle2
                    if(pos_turtle1_.theta <= 3.14 && pos_turtle1_.theta >= 1.57 &&
                        pos_turtle2_.theta <= 0.0 && pos_turtle2_.theta >= -1.57)
                        // Two turtles look at each other
                        relative_pos_.data = 2.0;
                    else if(pos_turtle1_.theta <= 3.14 && pos_turtle1_.theta >= 1.57)
                        // Turtle1 looks towards turtle2
                        relative_pos_.data = 2.1;
                    else if(pos_turtle2_.theta <= 0.0 && pos_turtle2_.theta >= -1.57)
                        // Turtle2 looks towards turtle1
                        relative_pos_.data = 2.2;
                }
            }
            else{
                if(pos_turtle1_.y >= pos_turtle2_.y){
                    // Turtle1 is at the top left relative to turtle2
                    if(pos_turtle1_.theta <= 0.0 && pos_turtle1_.theta >= -1.57 &&
                        pos_turtle2_.theta <= -3.14 && pos_turtle2_.theta >= 1.57)
                        // Two turtles look at each other
                        relative_pos_.data = 3.0;
                    else if(pos_turtle1_.theta <= 0.0 && pos_turtle1_.theta >= -1.57)
                        // Turtle1 looks towards turtle2
                        relative_pos_.data = 3.1;
                    else if(pos_turtle2_.theta <= -3.14 && pos_turtle2_.theta >= 1.57)
                        // Turtle2 looks towards turtle1
                        relative_pos_.data = 3.2;
                }
                else{
                    // Turtle1 is at the bottom left relative to turtle2
                    if(pos_turtle1_.theta >= 0.0 && pos_turtle1_.theta <= 1.57 &&
                        pos_turtle2_.theta >= -3.14 && pos_turtle2_.theta <= -1.57)
                        // Two turtles look at each other
                        relative_pos_.data = 4.0;
                    else if(pos_turtle1_.theta >= 0.0 && pos_turtle1_.theta <= 1.57)
                        // Turtle1 looks towards turtle2
                        relative_pos_.data = 4.1;
                    else if(pos_turtle2_.theta >= -3.14 && pos_turtle2_.theta <= -1.57)
                        // Turtle2 looks towards turtle1
                        relative_pos_.data = 4.2;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Rel_pos_: %f",relative_pos_.data);
            publisher4_->publish(relative_pos_);
        }

        rclcpp::TimerBase::SharedPtr timer1_,timer2_,timer3_,timer4_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher1_,publisher2_,publisher3_,publisher4_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_,subscription2_;
        std_msgs::msg::Float32 distance_,relative_pos_,boundary_condition_on_1_,boundary_condition_on_2_;
        turtlesim::msg::Pose pos_turtle1_,pos_turtle2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
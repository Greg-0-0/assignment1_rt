#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "turtlesim/msg/pose.hpp"
#include <iostream>
#include <string.h>
#include <math.h>

#define threshold 2.0

using std::placeholders::_1;

class UINode : public rclcpp::Node
{
    public:
        UINode() : Node("ui_node")
        {
            publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); // creates publisher on topic 'turtle1/cmd_vel'
            publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10); // creates publisher on topic 'turtle2/cmd_vel'
            subscription1_ = this->create_subscription<std_msgs::msg::Float32>("distance", 10,std::bind(&UINode::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<std_msgs::msg::Float32>("boundary_condition", 10,std::bind(&UINode::topic_callback2, this, _1));
            while(1){
                user_input();
                if(n_robot_ == 0)
                    exit(EXIT_SUCCESS);
                move_turtle_x();
                move_turtle_y();
            }
        }
    private:
        void user_input(){
            n_robot_ = 0; vel_x_ = 0.0; vel_y_ = 0.0;
            std::cout << "Select a robot to move it (type 1 or 2) or type 0 to quit:\n";
            std::cin >> n_robot_;
            if(n_robot_ == 0)
                return;
            std::cout << "Insert linear speed on x: \n";
            std::cin >> vel_x_;
            std::cout << "Insert linear speed on y: \n";
            std::cin >> vel_y_;
            return;
        }
        void move_turtle_x(){
            turtle_twist_.linear.x = vel_x_;
            if(n_robot_ == 1)
                publisher1_->publish(turtle_twist_);
            else
                publisher2_->publish(turtle_twist_);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback, this));
        }
        void move_turtle_y(){
            /*RCLCPP_INFO(this->get_logger(), "Distance: %f, abs distance: %f", distance_, fabs(distance_));
            if(fabs(distance_) < threshold){
                // Control before moving
                if(distance_ <= 0){
                    if((n_robot_ == 1 && vel_ > 0) || (n_robot_ == 2 && vel_ < 0))
                        std::cout << "Turtles too close, choose another turtle or change velocity\n";
                    else{
                        turtle_twist_.linear.y = vel_y_;
                        if(n_robot_ == 1)
                            publisher1_->publish(turtle_twist_);
                        else
                            publisher2_->publish(turtle_twist_);
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback, this));
                    }
                }
                else{
                    if((n_robot_ == 1 && vel_ < 0) || (n_robot_ == 2 && vel_ > 0))
                        std::cout << "Turtles too close, choose another turtle or change velocity\n";
                    else{
                        turtle_twist_.linear.y = vel_y_;
                        if(n_robot_ == 1)
                            publisher1_->publish(turtle_twist_);
                        else
                            publisher2_->publish(turtle_twist_);
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback, this));
                    }
                }
            }
            else{*/
            turtle_twist_.linear.y = vel_y_;
            if(n_robot_ == 1)
                publisher1_->publish(turtle_twist_);
            else
                publisher2_->publish(turtle_twist_);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback, this));
        }
        void timer_callback(){
            turtle_twist_.linear.x = 0.0;
            turtle_twist_.linear.y = 0.0;
            publisher1_->publish(turtle_twist_);
            publisher2_->publish(turtle_twist_);
            RCLCPP_INFO(this->get_logger(), "Stopping the turtle");
            timer_->cancel();
            return;
        }
        void topic_callback1(const std_msgs::msg::Float32::SharedPtr dis){
            // issue: understanding along which direction a turtle cannot move
            distance_ = dis->data;
            if(fabs(dis->data) < threshold){
                if(dis->data <= 0)
                // Turtle 1 
                turtle_twist_.linear.x = 0.0;
                publisher1_->publish(turtle_twist_);
                publisher2_->publish(turtle_twist_);
                RCLCPP_INFO(this->get_logger(), "Stopping the turtle");
                std::cout << "Turtle stopped (too close)\n";
                timer_->cancel();
            }
        }

        void topic_callback2(const std_msgs::msg::Float32::SharedPtr boundary_cond){
            if((boundary_cond->data == 1.1 || boundary_cond->data == 1.0 || boundary_cond->data == 1.2)
                && vel_x_ > 0)
                // Turtle1 reached right border x
                turtle_twist_.linear.x = 0.0;
            else if((boundary_cond->data == -1.1 || boundary_cond->data == -1.0 || boundary_cond->data == -1.2)
                && vel_x_ < 0)
                // Turtle1 reached left border x
                turtle_twist_.linear.x = 0.0;
            if((boundary_cond->data == 1.2 || boundary_cond->data == -1.0 || boundary_cond->data == 0.1 )
                && vel_y_ > 0)
                // Turtle1 reached upper border y
                turtle_twist_.linear.y = 0.0;
            else if((boundary_cond->data == 1.0 || boundary_cond->data == -1.2 || boundary_cond->data == -0.1 )
                && vel_y_ < 0)
                // Turtle1 reached upper border y
                turtle_twist_.linear.y = 0.0;
            publisher1_->publish(turtle_twist_);
            publisher2_->publish(turtle_twist_);
            RCLCPP_INFO(this->get_logger(), "Stopping the turtle: border reached");
            std::cout << "Stopping the turtle: border reached\n";
            timer_->cancel();
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription1_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription2_;
        geometry_msgs::msg::Twist turtle_twist_;
        int n_robot_; double vel_x_, vel_y_, distance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}
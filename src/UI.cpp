#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string.h>

class UINode : public rclcpp::Node
{
    public:
        UINode() : Node("ui_node")
        {
            publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); // creates publisher on topic 'turtle1/cmd_vel'
            publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10); // creates publisher on topic 'turtle2/cmd_vel'
            while(1){
                user_input();
                move_turtle();
            }
        }
    private:
        void user_input(){
            n_robot_ = 0; vel_ = 0;
            std::cout << "Select a robot to move it (type 1 or 2):\n";
            std::cin >> n_robot_;
            while(n_robot_ != 1 && n_robot_ != 2){
                std::cout << "Invalid number, choose again: \n";
                std::cin >> n_robot_;
            }
            std::cout << "Insert speed: \n";
            std::cin >> vel_;
            }
            return;
        }
        void move_turtle(){
            turtle_twist_.linear.x = vel_;
            if(n_robot_ == 1)
                publisher1_->publish(turtle_twist_);
            else
                publisher2_->publish(turtle_twist_);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback, this));
        }
        void timer_callback(){
            turtle_twist_.linear.x = 0.0;
            publisher1_->publish(turtle_twist_);
            publisher2_->publish(turtle_twist_);
            RCLCPP_INFO(this->get_logger(), "Stopping the turtle");
            timer_->cancel();
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
        geometry_msgs::msg::Twist turtle_twist_;
        int n_robot_, vel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}
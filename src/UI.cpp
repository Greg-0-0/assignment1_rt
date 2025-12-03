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
            subscription2_ = this->create_subscription<std_msgs::msg::Float32>("boundary_condition_on_1", 10,std::bind(&UINode::topic_callback2, this, _1));
            subscription3_ = this->create_subscription<std_msgs::msg::Float32>("boundary_condition_on_2", 10,std::bind(&UINode::topic_callback3, this, _1));
            subscription4_ = this->create_subscription<std_msgs::msg::Float32>("relative_pos", 10,std::bind(&UINode::topic_callback4, this, _1));
            main_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&UINode::main_loop, this));
        }
    private:
        void main_loop(){

            if(!waiting_for_input_) 
                return;

            user_input();
            if(n_robot_ == 0){
                rclcpp::shutdown();
                return;
            }
            rotate_turtle();
            waiting_for_input_ = false;
        }

        void user_input(){
            n_robot_ = 0; vel_x_ = 0.0; vel_ang_ = 0.0;
            std::cout << "Select a robot to move it (type 1 or 2) or type 0 to quit:\n";
            std::cin >> n_robot_;
            if(n_robot_ == 0)
                return;
            std::cout << "Insert angular speed: \n";
            std::cin >> vel_ang_;
            std::cout << "Insert linear speed on x: \n";
            std::cin >> vel_x_;
            return;
        }

        // Executes turtle rotation
        void rotate_turtle(){
            turtle_twist_.angular.z = vel_ang_;
            if(n_robot_ == 1)
                publisher1_->publish(turtle_twist_);
            else
                publisher2_->publish(turtle_twist_);
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback1, this));
        }

        // Stops turtle rotation after 1 second
        void timer_callback1(){
            turtle_twist_.angular.z = 0.0;
            publisher1_->publish(turtle_twist_);
            publisher2_->publish(turtle_twist_);
            timer1_->cancel();
            move_turtle_x();
            return;
        }

        // Applies linear velocity along x axis relative to local reference frame of the turtle
        void move_turtle_x(){
            bool motion_allowed = constraints_check();
            if(!motion_allowed){
                std::cout << "Minimum distance reached, choose another direction or velocity\n";
                waiting_for_input_ = true;
                return;
            }

            turtle_twist_.linear.x = vel_x_;
            if(n_robot_ == 1)
                publisher1_->publish(turtle_twist_);
            else
                publisher2_->publish(turtle_twist_);
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&UINode::timer_callback2, this));
        }
        
        // Triggers after 1 second from linear movement start to stop turtle translation
        void timer_callback2(){
            
            stop_linear_motion();
            timer2_->cancel();
            waiting_for_input_ = true;
        }

        // Stops turtle translation (need a specific function since it could even stop before the 1 second timer elapses)
        void stop_linear_motion(){
            turtle_twist_.linear.x = 0.0;
            publisher1_->publish(turtle_twist_);
            publisher2_->publish(turtle_twist_);
        }

        // Receives distance between the two turtles in real time
        void topic_callback1(const std_msgs::msg::Float32::SharedPtr dis){
            distance_ = dis->data;

            if(timer2_ && timer2_->is_canceled() == false){
                // Direction fixed and movement is in progress
                if(distance_ < threshold){
                    if(motion_would_collide()){
                        std::cout << "Stopping motion: minimum threshold reached\n";
                        stop_movement_in_progress();
                    }
                }
            }
        }

        // Retrieves condition on boundaries for turtle1
        void topic_callback2(const std_msgs::msg::Float32::SharedPtr boundary_cond_on_1){
            boundary_cond_on_1_ = boundary_cond_on_1->data;
            if(n_robot_ == 1 && timer2_ && !timer2_->is_canceled()){
                // Direction fixed and movement is in progress
                if(boundary_violation(1)){
                    std::cout << "Stopping motion: boundary has been reached\n";
                    stop_movement_in_progress();
                }
            }
        }

        // Retrieves condition on boundaries for turtle2
        void topic_callback3(const std_msgs::msg::Float32::SharedPtr boundary_cond_on_2){
            boundary_cond_on_2_ = boundary_cond_on_2->data;
            if(n_robot_ == 2 && timer2_ && !timer2_->is_canceled()){
                // Direction fixed and movement is in progress
                if(boundary_violation(2)){
                    std::cout << "Stopping motion: boundary has been reached\n";
                    stop_movement_in_progress();
                }
            }
        }

        // Retrieves condition on relative position and orientation of two turtles
        void topic_callback4(const std_msgs::msg::Float32::SharedPtr rel_pos){
            rel_pos_ = rel_pos->data;
        }

        // Called to stop translation after movement has started
        void stop_movement_in_progress(){
            // Motion is stopped
            stop_linear_motion();
            if(timer2_)
                timer2_->cancel();
            waiting_for_input_ = true; // ready for new input
        }

        // Checks for collision with other turtle
        bool constraints_check(){
            bool no_collision = true;
            // Checking collsion with other turtle
            if(distance_ < threshold){
                if(motion_would_collide()){
                    no_collision = false;
                }
            }

            // Checking collision with borders
            if(n_robot_ == 1 && boundary_violation(1))
                no_collision = false;
            if(n_robot_ == 2 && boundary_violation(2))
                no_collision = false;

            return no_collision;
        }

        // Given distance < threshold, checking relative position and orientation of the two turtles to detect possible future collisions
        bool motion_would_collide(){
            bool result = false;
            if((fabs(rel_pos_ - 1.0) < eps_ && vel_x_ > 0) || 
                (fabs(rel_pos_ - 1.1) < eps_ && ((n_robot_ == 1 && vel_x_ > 0) || (n_robot_ == 2 && vel_x_ < 0))) ||
                (fabs(rel_pos_ - 1.2) < eps_ && ((n_robot_ == 1 && vel_x_ < 0) || (n_robot_ == 2 && vel_x_ > 0))))
                // Turtle1 is at the top right relative to turtle2
                result = true;
            if((fabs(rel_pos_ - 2.0) < eps_ && vel_x_ > 0) || 
                (fabs(rel_pos_ - 2.1) < eps_ && ((n_robot_ == 1 && vel_x_ > 0) || (n_robot_ == 2 && vel_x_ < 0))) ||
                (fabs(rel_pos_ - 2.2) < eps_ && ((n_robot_ == 1 && vel_x_ < 0) || (n_robot_ == 2 && vel_x_ > 0))))
                // Turtle1 is at the bottom right relative to turtle2
                result = true;
            if((fabs(rel_pos_ - 3.0) < eps_ && vel_x_ > 0) || 
                (fabs(rel_pos_ - 3.1) < eps_ && ((n_robot_ == 1 && vel_x_ > 0) || (n_robot_ == 2 && vel_x_ < 0))) ||
                (fabs(rel_pos_ - 3.2) < eps_ && ((n_robot_ == 1 && vel_x_ < 0) || (n_robot_ == 2 && vel_x_ > 0))))
                // Turtle1 is at the top left relative to turtle2
                result = true;
            if((fabs(rel_pos_ - 4.0) < eps_ && vel_x_ > 0) || 
                (fabs(rel_pos_ - 4.1) < eps_ && ((n_robot_ == 1 && vel_x_ > 0) || (n_robot_ == 2 && vel_x_ < 0))) ||
                (fabs(rel_pos_ - 4.2) < eps_ && ((n_robot_ == 1 && vel_x_ < 0) || (n_robot_ == 2 && vel_x_ > 0)))){
                    // Turtle1 is at the top left relative to turtle2
                    result = true;
                }
            return result;
        }

        // Checks if a turtle reached a border
        bool boundary_violation(int n_robot){
            double val = (n_robot == 1 ? boundary_cond_on_1_ : boundary_cond_on_2_);

            if((fabs(val - 1.0) < eps_ && vel_x_ > 0) || (fabs(val - 1.5) < eps_ && vel_x_ < 0))
                // Right boundary reached
                return true;
            if((fabs(val - 2.0) < eps_ && vel_x_ > 0) || (fabs(val - 2.5) < eps_ && vel_x_ < 0))
                // Right boundary reached
                return true;
            if((fabs(val - 3.0) < eps_ && vel_x_ > 0) || (fabs(val - 3.5) < eps_ && vel_x_ < 0))
                // Right boundary reached
                return true;
            if((fabs(val - 4.0) < eps_ && vel_x_ > 0) || (fabs(val - 4.5) < eps_ && vel_x_ < 0))
                // Right boundary reached
                return true;
            return false;
        }

        rclcpp::TimerBase::SharedPtr timer1_,timer2_,main_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_,publisher2_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription1_,subscription2_,subscription3_,subscription4_;
        geometry_msgs::msg::Twist turtle_twist_;
        double vel_x_, vel_ang_, rel_pos_, distance_, boundary_cond_on_1_, boundary_cond_on_2_, eps_ = 1e-3;
        bool waiting_for_input_ = true;
        int n_robot_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <iostream>
#include <string.h>

using std::placeholders::_1;

class DistanceNode : public rclcpp::Node
{
    public:
        DistanceNode() : Node("distance_node")
        {
            publisher_ = this->create_publisher<turtlesim::msg::Pose>("distances", 10); // to publish ditances between turtle1 and turtle2
            subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,std::bind(&DistanceNode::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,std::bind(&DistanceNode::topic_callback2, this, _1));
        }
    private:
        void topic_callback1(const turtlesim::msg::Pose::SharedPtr pos){
            RCLCPP_INFO(this->get_logger(), "Turtle1 updated: %f", pos->x);
        }
        void topic_callback2(const turtlesim::msg::Pose::SharedPtr pos){
            RCLCPP_INFO(this->get_logger(), "Turtle2 updated: %f",pos->x);
        }
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
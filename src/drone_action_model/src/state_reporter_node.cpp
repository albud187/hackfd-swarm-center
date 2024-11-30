// Libraries
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
class PoseNode : public rclcpp::Node
{
public:

    PoseNode() : Node("pose_node")
    {
        std::string ns = this->get_namespace();
        std::string T_pose = ns + "/gt_pose";
        std::string T_RPY_pose = ns + "/RPY_pose";
        pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            T_pose, 10, std::bind(&PoseNode::pose_callback, this, std::placeholders::_1));

        RPY_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            T_RPY_pose, 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr RPY_pose_pub;
    geometry_msgs::msg::PoseStamped pose_msg;

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        
        pose_msg.pose.position = msg->position;

        // Extract the quaternion and convert to yaw
        tf2::Quaternion quat;
        tf2::fromMsg(msg->orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        tf2::Quaternion pose_quat;
        pose_quat.setRPY(roll, pitch, yaw);
        pose_msg.pose.orientation.x = roll;
        pose_msg.pose.orientation.y = pitch;
        pose_msg.pose.orientation.z = yaw;

        pose_msg.header.stamp = rclcpp::Node::now();
        RPY_pose_pub->publish(pose_msg);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<PoseNode>();
    rclcpp::spin(rclcppNode);
    rclcpp::shutdown();
    return 0;
}
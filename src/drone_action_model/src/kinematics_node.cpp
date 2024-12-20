#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <cmath>

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node")
    {
        std::string ns = this->get_namespace();
        std::string T_velocity_vector = ns + "/cmd_vel_vector";
        std::string T_RPY_pose = ns + "/RPY_pose";
        std::string T_cmd_vel = ns + "/cmd_vel";

        velocity_vector_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            T_velocity_vector, 10, std::bind(&KinematicsNode::velocity_vector_callback, this, std::placeholders::_1));

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            T_RPY_pose, 10, std::bind(&KinematicsNode::RPY_pose_callback, this, std::placeholders::_1));

        velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>(T_cmd_vel, 60);

        velocity_pub_thread = std::thread(&KinematicsNode::publish_velocity_command, this);
    }

    void join_velocity_pub_thread()
    {
        if (velocity_pub_thread.joinable()){
            velocity_pub_thread.join();
        }
    }

private:
    std::mutex mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_vector_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;

    std::thread velocity_pub_thread;

    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Vector3 velocity_vector;

    void RPY_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose = *msg;
    }

    void velocity_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_vector = *msg;
    }

    float normalize_angle(float angle)
    {
        while (angle > M_PI){
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI){
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    geometry_msgs::msg::Twist get_command_velocity(
        geometry_msgs::msg::Vector3 vel_vector,
        geometry_msgs::msg::PoseStamped drone_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;
       
        float Vxy = std::sqrt(vel_vector.x * vel_vector.x + vel_vector.y * vel_vector.y);
        float heading = std::atan2(vel_vector.y, vel_vector.x);
        float d_yaw = normalize_angle(heading - drone_pose.pose.orientation.z);

        cmd_vel.linear.x = Vxy * std::cos(d_yaw);
        cmd_vel.linear.y = Vxy * std::sin(d_yaw);
        cmd_vel.linear.z = vel_vector.z;

        if (Vxy < 1.5){
            cmd_vel.angular.z = 0.0;
        }
        else{
            cmd_vel.angular.z = 2.0 * d_yaw + 0.5 * (d_yaw - 0.0);
        }

        return cmd_vel;
    }

    void publish_velocity_command()
    {
        while (rclcpp::ok()){
            geometry_msgs::msg::Twist command_velocity = get_command_velocity(velocity_vector, current_pose);
            velocity_pub->publish(command_velocity);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<KinematicsNode>();

    rclcpp::spin(rclcppNode);

    rclcppNode->join_velocity_pub_thread();

    rclcpp::shutdown();
    return 0;
}

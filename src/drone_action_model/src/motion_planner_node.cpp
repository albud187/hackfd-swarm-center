#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <cmath>

class MotionPlannerNode : public rclcpp::Node
{
public:
    MotionPlannerNode() : Node("motion_planner_node")
    {
        std::string ns = this->get_namespace();
        std::string T_goal_pose = ns + "/goal_pose";
        std::string T_RPY_pose = ns + "/RPY_pose";
        std::string T_velocity_vector = ns + "/cmd_vel_vector";

        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            T_goal_pose, 10, std::bind(&MotionPlannerNode::goal_pose_callback, this, std::placeholders::_1));

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            T_RPY_pose, 10, std::bind(&MotionPlannerNode::pose_callback, this, std::placeholders::_1));

        velocity_vector_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_velocity_vector, 10);

        velocity_vector_pub_thread = std::thread(&MotionPlannerNode::publish_velocity_vector, this);
    }

    void join_velocity_pub_thread()
    {
        if (velocity_vector_pub_thread.joinable())
        {
            velocity_vector_pub_thread.join();
        }
    }

private:
    std::mutex mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_vector_pub;

    std::thread velocity_vector_pub_thread;

    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Vector3 velocity_vector;

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_pose = *msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose = *msg;
    }

    geometry_msgs::msg::Vector3 calculate_goal_velocity(
        geometry_msgs::msg::PoseStamped goal_pose,
        geometry_msgs::msg::PoseStamped current_pose,
        float vxy_max, float vz_max, float slow_radius){
        
        float dt = 1.0;
        geometry_msgs::msg::Vector3 result;

        float dx = goal_pose.pose.position.x - current_pose.pose.position.x;
        float dy = goal_pose.pose.position.y - current_pose.pose.position.y;
        float dz = goal_pose.pose.position.z - current_pose.pose.position.z;

        float d_magxy = std::sqrt(dx * dx + dy * dy);
        float scale_xy = 1.0;

        if (d_magxy*dt < slow_radius){
            scale_xy = d_magxy / slow_radius;
        }

        float target_vxy = vxy_max * scale_xy;

        if (d_magxy*dt > 0){
            result.x = target_vxy * dx / d_magxy;
            result.y = target_vxy * dy / d_magxy;
        }else{
            result.x = 0.0;
            result.y = 0.0;
        }

        if (std::abs(dz*dt) > vz_max){
            if (dz > 0){
                result.z = vz_max;
            }else{
                result.z = -vz_max;
            }
        }else{
            result.z = dz;
        }

        return result;
    }

    geometry_msgs::msg::Vector3 ramp_velocity(
        geometry_msgs::msg::Vector3 current_velocity,
        geometry_msgs::msg::Vector3 desired_velocity,
        float max_acceleration)
    {
        geometry_msgs::msg::Vector3 result;

        //acceleration
        float ax = desired_velocity.x - current_velocity.x;
        float ay = desired_velocity.y - current_velocity.y;
        float az = desired_velocity.z - current_velocity.z;

        float a_mag = std::sqrt(ax * ax + ay * ay + az * az);

        if (a_mag > max_acceleration){
            float scale = max_acceleration / a_mag;
            ax *= scale;
            ay *= scale;
            az *= scale;
        }

        result.x = current_velocity.x + ax;
        result.y = current_velocity.y + ay;
        result.z = current_velocity.z + az;

        return result;
    }

    void publish_velocity_vector()
    {
        while (rclcpp::ok())
        {
            geometry_msgs::msg::PoseStamped local_goal_pose;
            geometry_msgs::msg::PoseStamped local_current_pose;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                local_goal_pose = goal_pose;
                local_current_pose = current_pose;
            }

            if (local_goal_pose.header.frame_id == "1"){
                geometry_msgs::msg::Vector3 desired_velocity = calculate_goal_velocity(
                    local_goal_pose, local_current_pose, 5.0, 5.0, 5.0);
                velocity_vector = ramp_velocity(velocity_vector, desired_velocity, 1.0);
                velocity_vector_pub->publish(velocity_vector);
            }

            if (local_goal_pose.header.frame_id == "2"){
                geometry_msgs::msg::Vector3 desired_velocity = calculate_goal_velocity(
                    local_goal_pose, local_current_pose, 15.0, 5.0, 5.0);
                velocity_vector = ramp_velocity(velocity_vector, desired_velocity, 1.0);
                velocity_vector_pub->publish(velocity_vector);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<MotionPlannerNode>();

    rclcpp::spin(rclcppNode);

    rclcppNode->join_velocity_pub_thread();

    rclcpp::shutdown();
    return 0;
}

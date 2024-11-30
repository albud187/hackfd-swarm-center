#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <mutex>
#include <thread>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


class UserCameraNode : public rclcpp::Node
{
public:

    UserCameraNode() : Node("user_camera_node"){
        std::string ns = this->get_namespace();
        std::string T_cam_img = ns + "/front/image_raw";

        cam_ui_sub = this->create_subscription<sensor_msgs::msg::Image>(
            T_cam_img, 10, std::bind(&UserCameraNode::camera_image_cb, this, std::placeholders::_1));

    }

private:
    std::mutex mutex_;
    std::string image_tile = "/front/image_raw";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_ui_sub;

    sensor_msgs::msg::Image camera_image;
   
    cv::Mat target_image;


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<UserCameraNode>();

    rclcpp::spin(rclcppNode);

    rclcpp::shutdown();
    return 0;
}
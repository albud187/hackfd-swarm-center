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
    double scale_factor = 0.6;
    std::string ns = this->get_namespace();
    std::string image_title = ns + "/front/image_raw";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_ui_sub;

    sensor_msgs::msg::Image camera_image;
   
    cv::Mat target_image;

    void camera_image_cb(const sensor_msgs::msg::Image::SharedPtr msg){
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat resized_image;
            cv::resize(cv_ptr->image, resized_image, cv::Size(), scale_factor, scale_factor);

            cv::imshow(image_title, resized_image);
            cv::waitKey(1);
        }catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<UserCameraNode>();

    rclcpp::spin(rclcppNode);

    rclcpp::shutdown();
    return 0;
}
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "message_custom/msg/goal_frame.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class FramesPublisher : public rclcpp::Node{
    public:
        FramesPublisher() : Node("tf2_frames_publisher"){

            //PARAMETER
            goal_frame_ = this->declare_parameter<std::string>("frame_name", "goal_frame");

            static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            auto handle_goal_pose = [this](const std::shared_ptr<const message_custom::msg::GoalFrame> msg){
                geometry_msgs::msg::TransformStamped t2;

                // Read message content and assign it to
                // corresponding tf variables
                t2.header.stamp = this->get_clock()->now();
                t2.header.frame_id = "odom";
                t2.child_frame_id = goal_frame_;

                // Turtle only exists in 2D, thus we get x and y translation
                // coordinates from the message and set the z coordinate to 0
                t2.transform.translation.x = msg->x_goal;
                t2.transform.translation.y = msg->y_goal;
                t2.transform.translation.z = 0.0;

                // For the same reason, turtle can only rotate around one axis
                // and this why we set rotation in x and y to 0 and obtain
                // rotation in z axis from the message
                tf2::Quaternion q;
                q.setRPY(0, 0, msg->theta_goal);
                t2.transform.rotation.x = q.x();
                t2.transform.rotation.y = q.y();
                t2.transform.rotation.z = q.z();
                t2.transform.rotation.w = q.w();

                // Send the transformation
                static_broadcaster_->sendTransform(t2);
            };

            goal_frame_sub_ = this->create_subscription<message_custom::msg::GoalFrame>("/goal_frame", 10, handle_goal_pose);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&FramesPublisher::odom_callback, this, std::placeholders::_1));
        }

    private:

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped t1;

            t1.header.stamp = this->get_clock()->now();
            t1.header.frame_id = "odom"; 
            t1.child_frame_id = "base_footprint";

            t1.transform.translation.x = msg->pose.pose.position.x;
            t1.transform.translation.y = msg->pose.pose.position.y;
            t1.transform.translation.z = msg->pose.pose.position.z;

            t1.transform.rotation = msg->pose.pose.orientation;

            tf_broadcaster_->sendTransform(t1);
        }

        //PARAMETER
        std::string goal_frame_;
    
        //SUBRSCRIBER
        rclcpp::Subscription<message_custom::msg::GoalFrame>::SharedPtr goal_frame_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        //BROADCASTER
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramesPublisher>());
    rclcpp::shutdown();
    return 0;
}
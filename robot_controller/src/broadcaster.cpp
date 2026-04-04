#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "message_custom/msg/goal_frame.hpp"

class FramePublisher : public rclcpp::Node{
    public:
        FramePublisher() : Node("goal_tf2_frame_publisher"){

            // Initialize the transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            auto handle_goal_pose = [this](const std::shared_ptr<const message_custom::msg::GoalFrame> msg){
                geometry_msgs::msg::TransformStamped t;

                // Read message content and assign it to
                // corresponding tf variables
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "world";
                t.child_frame_id = msg->name_frame;

                // Turtle only exists in 2D, thus we get x and y translation
                // coordinates from the message and set the z coordinate to 0
                t.transform.translation.x = msg->x_goal;
                t.transform.translation.y = msg->y_goal;
                t.transform.translation.z = 0.0;

                // For the same reason, turtle can only rotate around one axis
                // and this why we set rotation in x and y to 0 and obtain
                // rotation in z axis from the message
                tf2::Quaternion q;
                q.setRPY(0, 0, msg->theta_goal);
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                // Send the transformation
                tf_broadcaster_->sendTransform(t);
            };

            goal_frame_sub_ = this->create_subscription<message_custom::msg::GoalFrame>("/goal_frame", 10, handle_goal_pose);
        }

    private:
    
        //SUBRSCRIBER
        rclcpp::Subscription<message_custom::msg::GoalFrame>::SharedPtr goal_frame_sub_;

        //BROADCASTER
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
#ifndef ROBOT_CONTROLLER__ROBOT_ACTION_CLIENT_HPP_
#define ROBOT_CONTROLLER__ROBOT_ACTION_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "action_interfaces/action/target.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "std_msgs/msg/string.hpp"


namespace robot_controller {
    class RobotActionClient : public rclcpp::Node{
        public:
            using Target = action_interfaces::action::Target;
            using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

            explicit RobotActionClient(const rclcpp::NodeOptions & options);
            
            void send_goal(double x_goal, double y_goal, double theta_goal);

        private:

            void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg);

            void goal_topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

            void goal_response_callback(GoalHandleTarget::SharedPtr goal_handle);

            void feedback_callback(GoalHandleTarget::SharedPtr, const std::shared_ptr<const Target::Feedback> feedback);

            void result_callback(const GoalHandleTarget::WrappedResult & result);

            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_sub;
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
            rclcpp_action::Client<Target>::SharedPtr action_client_;
            std::string world_frame_name_;
            std::string goal_frame_name_;
            bool cancel_sent_;
    };
}

#endif
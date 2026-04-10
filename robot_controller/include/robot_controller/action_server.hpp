#ifndef ROBOT_CONTROLLER__ROBOT_ACTION_SERVER_HPP_
#define ROBOT_CONTROLLER__ROBOT_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

#include "action_interfaces/action/target.hpp"

namespace robot_controller {

    class RobotActionServer : public rclcpp::Node{

        public:
            using Target = action_interfaces::action::Target;
            using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

            explicit RobotActionServer(const rclcpp::NodeOptions & options);

        private:

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Target::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTarget> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle);
            
            void stop_robot();

            void execute(const std::shared_ptr<GoalHandleTarget> goal_handle);
        
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub{nullptr};
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            rclcpp_action::Server<Target>::SharedPtr action_server_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            geometry_msgs::msg::Twist velocity;
            std::string moved_frame_;
            std::string world_frame_;
            std::string goal_frame_;
            
    };
}

#endif
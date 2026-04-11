#include <chrono>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_controller/action_server.hpp"

#include "tf2/exceptions.hpp"



using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace robot_controller{

    RobotActionServer::RobotActionServer(const rclcpp::NodeOptions & options) : Node("robot_action_server", options){

        //PUBLISHERS
        robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        //SUBSCRIBERS
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RobotActionServer::odom_callback, this, std::placeholders::_1));

        //PARAMETERS
        goal_frame_ = this->declare_parameter<std::string>("target_frame_name", "goal_frame");
        moved_frame_ = this->declare_parameter<std::string>("moved_frame_name", "base_link");
        world_frame_ = this->declare_parameter<std::string>("world_frame_name", "odom");

        //BROADCASTER
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        //initialization odom frame to visualize it when RVIZ2 is opened the first time
        geometry_msgs::msg::TransformStamped t_init;
        t_init.header.stamp = this->get_clock()->now();
        t_init.header.frame_id = world_frame_;  
        t_init.child_frame_id = moved_frame_;    
        t_init.transform.rotation.w = 1.0;      
        tf_broadcaster_->sendTransform(t_init);
        
        //ACTION
        action_server_ = rclcpp_action::create_server<Target>(
            this,
            "target",
            std::bind(&RobotActionServer::handle_goal, this, _1, _2),
            std::bind(&RobotActionServer::handle_cancel, this, _1),
            std::bind(&RobotActionServer::handle_accepted, this, _1));

        //LISTENER
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void RobotActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = world_frame_; 
        t.child_frame_id = moved_frame_;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp_action::GoalResponse RobotActionServer::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Target::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "Received Goal: %.2f, %.2f, %.2f", goal->goal_pose[0], goal->goal_pose[1], goal->goal_pose[2]);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse RobotActionServer::handle_cancel(const std::shared_ptr<GoalHandleTarget> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void RobotActionServer::handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle){
        std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
    }

    void RobotActionServer::stop_robot(){
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        robot_vel_pub->publish(velocity);
    }


    void RobotActionServer::execute(const std::shared_ptr<GoalHandleTarget> goal_handle){
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Target::Feedback>();
        auto result = std::make_shared<Target::Result>();

        velocity.linear.x = 1.0;

        rclcpp::Rate rate(10);

        while (rclcpp::ok()){

            // CHECK CANCEL
            if (goal_handle->is_canceling()) {

                stop_robot();
                goal_handle->canceled(result);

                RCLCPP_INFO(this->get_logger(), "Goal canceled!");

                return;
            }

            geometry_msgs::msg::TransformStamped t;

            try {
            t = tf_buffer_->lookupTransform(
                moved_frame_,
                goal_frame_,
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN( this->get_logger(), "Not transform: %s",ex.what());
                rate.sleep();
                continue;
            }

            static const double scaleRotationRate = 1.0;
            static const double scaleForwardSpeed = 0.5;

            float x_ = t.transform.translation.x;
            float y_ = t.transform.translation.y;
            double distance = sqrt(pow(x_, 2) + pow(y_, 2));
            
            if (distance < 0.01) {
                stop_robot();
                result->final_pose={x_, y_, 0.0};
                goal_handle->succeed(result);
                return;
            }

            velocity.angular.z = scaleRotationRate * atan2(y_,x_);
            velocity.linear.x = scaleForwardSpeed * sqrt(pow(x_, 2) +pow(y_, 2));
            robot_vel_pub->publish(velocity);

            feedback->current_pose = {x_, y_, distance};
            goal_handle->publish_feedback(feedback);
            rate.sleep();
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::RobotActionServer)

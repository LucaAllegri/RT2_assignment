#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "action_interfaces/action/target.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot_controller{

    class RobotActionServer: public rclcpp::Node{
        public:
            using Target = action_interfaces::action::Target;
            using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

            explicit RobotActionServer(const rclcpp::NodeOptions & options) : Node("robot_action_server", options){
                //PUBLISHERS
                robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
                
                //SUBSCRIBERS
                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RobotActionServer::odom_callback, this, _1));
                
                //ACTION
                action_server_ = rclcpp_action::create_server<Target>(
                    this,
                    "target",
                    std::bind(&RobotActionServer::handle_goal, this, _1, _2),
                    std::bind(&RobotActionServer::handle_cancel, this, _1),
                    std::bind(&RobotActionServer::handle_accepted, this, _1));

            }

        private:

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Target::Goal> goal){
                RCLCPP_INFO(this->get_logger(), "Received Goal: %.2f", goal->goal_pose[0]);

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTarget> goal_handle){
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_accepted(const std::shared_ptr<GoalHandleTarget> goal_handle){
                // this needs to return quickly to avoid blocking the executor, so spin up a new thread
                std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
            }

            void execute(const std::shared_ptr<GoalHandleTarget> goal_handle){
                auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<Target::Feedback>();
                auto result = std::make_shared<Target::Result>();

                double target_x = goal->goal_pose[0];

                velocity.linear.x = 1.0;

                rclcpp::Rate rate(10);

                while (rclcpp::ok() && current_x_ < target_x){

                    // CHECK CANCEL
                    if (goal_handle->is_canceling()) {

                        velocity.linear.x = 0.0;
                        robot_vel_pub->publish(velocity);

                        result->final_pose = {current_x_};
                        goal_handle->canceled(result);

                        RCLCPP_INFO(this->get_logger(), "Goal canceled!");

                        return;
                    }

                    robot_vel_pub->publish(velocity);

                    feedback->current_pose = {current_x_};
                    goal_handle->publish_feedback(feedback);

                    rate.sleep();
                }
            }

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
                current_x_ = msg->pose.pose.position.x;
            }

            //PUBLISHER
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;

            //SUBSCRIBERS
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            
            //ACTION
            rclcpp_action::Server<Target>::SharedPtr action_server_;

            //VARIABLES
            geometry_msgs::msg::Twist velocity;
            float current_x_;
            
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::RobotActionServer)


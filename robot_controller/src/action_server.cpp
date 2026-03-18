#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "action_interfaces/action/movement.hpp"
#include <math.h>
using std::placeholders::_1;
using std::placeholders::_2;

class RobotActionServer: public rclcpp::Node{
    public:
        using Movement = action_interfaces::action::Movement;
        using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

        RobotActionServer(): Node("robot_action_server"){

            //PUBLISHERS
            robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            
            //SUBSCRIBERS
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RobotActionServer::odom_callback, this, _1));
            
            //ACTION
            action_server_ = rclcpp_action::create_server<Movement>(
                this,
                "movement",
                std::bind(&RobotActionServer::handle_goal, this, _1, _2),
                std::bind(&RobotActionServer::handle_cancel, this, _1),
                std::bind(&RobotActionServer::handle_accepted, this, _1));

        }

    private:

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Movement::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received Goal: %.2f", goal->goal_position[0]);

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle){
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleMovement> goal_handle){
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Movement::Feedback>();
            auto result = std::make_shared<Movement::Result>();

            double target_x = goal->goal_position[0];

            velocity.linear.x = 1.0;

            rclcpp::Rate rate(10);

            while (rclcpp::ok() && current_x_ < target_x){
                robot_vel_pub->publish(velocity);

                feedback->current_position = {current_x_};
                goal_handle->publish_feedback(feedback);

                rate.sleep();
            }

            // STOP robot
            velocity.linear.x = 0.0;
            robot_vel_pub->publish(velocity);

            result->final_position = {current_x_};
            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "Goal raggiunto!");
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            current_x_ = msg->pose.pose.position.x;
        }

        //PUBLISHER
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;

        //SUBSCRIBERS
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        
        //ACTION
        rclcpp_action::Server<Movement>::SharedPtr action_server_;

        //VARIABLES
        geometry_msgs::msg::Twist velocity;
        float current_x_;
        
};


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    // Crea e lancia il nodo server
    auto node = std::make_shared<RobotActionServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

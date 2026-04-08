#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

#include "action_interfaces/action/target.hpp"

#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace robot_controller{

    class RobotActionServer: public rclcpp::Node{
        public:
            using Target = action_interfaces::action::Target;
            using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

            //RobotActionServer() : Node("robot_action_server"){
            explicit RobotActionServer(const rclcpp::NodeOptions & options) : Node("robot_action_server", options){
                
                //PUBLISHERS
                robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

                //PARAMETERS
                goal_frame_ = this->declare_parameter<std::string>("target_frame_name", "goal_frame");
                moved_frame_ = this->declare_parameter<std::string>("moved_frame_name", "base_link");
                
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

        private:

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Target::Goal> goal){
                RCLCPP_INFO(this->get_logger(), "Received Goal: %.2f, %.2f, %.2f", goal->goal_pose[0], goal->goal_pose[1], goal->goal_pose[2]);

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

            void stop_robot(){
                velocity.linear.x = 0.0;
                velocity.angular.z = 0.0;
                robot_vel_pub->publish(velocity);
            }


            void execute(const std::shared_ptr<GoalHandleTarget> goal_handle){
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

                    // Look up for the transformation between target_frame and turtle2 frames
                    // and send velocity commands for turtle2 to reach target_frame
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

                    velocity.angular.z = scaleRotationRate * atan2(
                    y_,
                    x_);

                    velocity.linear.x = scaleForwardSpeed * sqrt(
                    pow(x_, 2) +
                    pow(y_, 2));

                    robot_vel_pub->publish(velocity);

                    double distance = sqrt(
                        pow(x_, 2) +
                        pow(y_, 2));
                    
                    if (distance < 0.01) {
                        stop_robot();
                        result->final_pose={x_, y_, 0.0};
                        goal_handle->succeed(result);
                        return;
                    }else{
                        feedback->current_pose = {x_, y_, 0.0};
                        goal_handle->publish_feedback(feedback);
                    }

                    rate.sleep();
                }
            }


            //PUBLISHER
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub{nullptr};
            
            //ACTION
            rclcpp_action::Server<Target>::SharedPtr action_server_;

            //LISTENER
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            //VARIABLES
            geometry_msgs::msg::Twist velocity;

            //PARAMETERS
            std::string moved_frame_;
            std::string goal_frame_;
    };
}

/*int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotActionServer>());
    rclcpp::shutdown();
    return 0;
}*/

RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::RobotActionServer)


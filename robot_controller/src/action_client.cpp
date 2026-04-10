#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include "robot_controller/action_client.hpp"

using namespace std::placeholders;


namespace robot_controller{

    RobotActionClient::RobotActionClient(const rclcpp::NodeOptions & options) : Node("robot_action_client", options), cancel_sent_(false) {

        action_client_ = rclcpp_action::create_client<Target>(this, "target");

        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        goal_frame_name_ = this->declare_parameter<std::string>("target_frame_name", "goal_frame");
        world_frame_name_ = this->declare_parameter<std::string>("world_frame_name", "odom");
    }

    void RobotActionClient::send_goal(double x_goal, double y_goal, double theta_goal){
        if (!action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = world_frame_name_;
        t.child_frame_id = goal_frame_name_;

        t.transform.translation.x = x_goal;
        t.transform.translation.y = y_goal;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_goal);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(t);

        cancel_sent_ = false;

        auto goal_msg = Target::Goal();
        goal_msg.goal_pose = {x_goal, y_goal, theta_goal};

        rclcpp_action::Client<Target>::SendGoalOptions options;

        options.goal_response_callback = std::bind(&RobotActionClient::goal_response_callback, this, _1);

        options.feedback_callback =std::bind(&RobotActionClient::feedback_callback, this, _1, _2);

        options.result_callback = std::bind(&RobotActionClient::result_callback, this, _1);

        action_client_->async_send_goal(goal_msg, options); 
    }

    void RobotActionClient::goal_response_callback(GoalHandleTarget::SharedPtr goal_handle){
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    }

    void RobotActionClient::feedback_callback(GoalHandleTarget::SharedPtr,const std::shared_ptr<const Target::Feedback> feedback){
        RCLCPP_INFO(this->get_logger(),"Feedback: remaining x = %f, remaining y = %f",feedback->current_pose[0], feedback->current_pose[1]);
    }

    void RobotActionClient::result_callback(const GoalHandleTarget::WrappedResult & result){
        RCLCPP_INFO(this->get_logger(),"Result status=%d, x_=%f, y_= %f",result.code,result.result->final_pose[0],result.result->final_pose[1]);
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::RobotActionClient)



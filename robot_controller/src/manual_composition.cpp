#include <cinttypes>
#include <cstdio>
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "action_interfaces/action/target.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

#include "action_client.cpp" 
#include "action_server.cpp"

using std::placeholders::_1;
using Target = action_interfaces::action::Target;
using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

/*
namespace robot_controller{

    class RobotActionClient : public rclcpp::Node{
        public:
            explicit RobotActionClient(const rclcpp::NodeOptions & options);
            
            void send_goal(double x_goal, double y_goal, double theta_goal);

        private:
            void goal_response_callback(GoalHandleTarget::SharedPtr goal_handle);

            void feedback_callback(GoalHandleTarget::SharedPtr, const std::shared_ptr<const Target::Feedback> feedback);

            void result_callback(const GoalHandleTarget::WrappedResult & result);

            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
            rclcpp_action::Client<Target>::SharedPtr action_client_;
            std::string world_frame_name_;
            std::string goal_frame_name_;
            bool cancel_sent_;
    };

    class RobotActionServer : public rclcpp::Node{
        public:
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
            std::string goal_frame_;
            std::string world_frame_;
    };
}  */

void check_input(double &i) { // Passaggio per riferimento necessario per aggiornare il valore!
    while (!(std::cin >> i)) {
        std::cout << "Input non valido. Inserisci un numero: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    double x_target=0.0, y_target=0.0, theta_target=0.0;

    auto talker = std::make_shared<robot_controller::RobotActionClient>(options);
    exec.add_node(talker);
    auto listener = std::make_shared<robot_controller::RobotActionServer>(options);
    exec.add_node(listener);

    std::cout << "insert x target: ";
    check_input(x_target);

    std::cout << "insert y target: ";
    check_input(y_target);

    std::cout << "insert theta target: ";
    check_input(theta_target);

    talker->send_goal(x_target,y_target,theta_target);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
#include <memory>

#include "robot_controller/action_client"
#include "robot_controller/action_service"
#include "robot_controller/broadcaster_component"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto talker = std::make_shared<robot_controller::RobotActionClient>(options);
    exec.add_node(talker);

    double x_target, y_target, theta_target;

    std::cout << "insert x target: ";
    if (!(std::cin >> x_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    std::cout << "insert y target: ";
    if (!(std::cin >> y_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    std::cout << "insert theta target: ";
    if (!(std::cin >> theta_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    talker->send_goal(x_target,y_target,theta_target);

    auto broadcaster = std::make_shared<robot_controller::FramesPublisher>(options);
    exec.add_node(broadcaster);

    auto listener = std::make_shared<robot_controller::RobotActionServer>(options);
    exec.add_node(listener);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
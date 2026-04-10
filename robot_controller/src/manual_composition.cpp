#include <memory>
#include "robot_controller/action_client.hpp"
#include "robot_controller/action_server.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
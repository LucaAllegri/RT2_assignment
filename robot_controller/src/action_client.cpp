#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/movement.hpp"

class RobotActionClient : public rclcpp::Node{
public:
    using Movement = action_interfaces::action::Movement;

    RobotActionClient() : Node("robot_action_client"){
        action_client_ = rclcpp_action::create_client<Movement>(this, "movement");

        action_client_->wait_for_action_server();

        std::thread(&RobotActionClient::input_loop, this).detach();

    }

private:

    void send_goal(float x_value){
        auto goal_msg = Movement::Goal();
        goal_msg.goal_position = {x_value};

        action_client_->async_send_goal(goal_msg);
    }

    void input_loop(){
        while (rclcpp::ok()){
            std::cout << "Insert x Goal: ";
            if (!(std::cin >> x_input)) {
                std::cout << "Invalid value input for x Goal.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            send_goal(x_input);
        }
    }

    //CLIENT
    rclcpp_action::Client<Movement>::SharedPtr action_client_;

    //VARIABLES
    float x_input;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    // Crea il nodo client
    auto node = std::make_shared<RobotActionClient>();
    
    // Spin per qualche tempo per inviare goal e ricevere feedback
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/movement.hpp"

class RobotActionClient : public rclcpp::Node{
    public:
        using Movement = action_interfaces::action::Movement;
        using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

        RobotActionClient() : Node("robot_action_client"), cancel_sent_(false){
            action_client_ = rclcpp_action::create_client<Movement>(this, "movement");
        }

        void send_goal(double x_target){
            if (!action_client_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available");
                return;
            }

            cancel_sent_ = false;

            auto goal_msg = Movement::Goal();
            goal_msg.goal_position = {x_target};

            using namespace std::placeholders;

            rclcpp_action::Client<Movement>::SendGoalOptions options;

            options.goal_response_callback = std::bind(&RobotActionClient::goal_response_callback, this, _1);

            options.feedback_callback =std::bind(&RobotActionClient::feedback_callback, this, _1, _2);

            options.result_callback = std::bind(&RobotActionClient::result_callback, this, _1);

            action_client_->async_send_goal(goal_msg, options); 
        }

    private:

        void goal_response_callback(GoalHandleMovement::SharedPtr goal_handle){
            if (!goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Goal rejected");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Goal accepted");
            current_goal_handle_ = goal_handle;
        }

        void feedback_callback(GoalHandleMovement::SharedPtr,const std::shared_ptr<const Movement::Feedback> feedback){
            double remaining = feedback->current_position[0];

            RCLCPP_INFO(this->get_logger(),"Feedback: remaining angle = %f",remaining);

            if (cancel_sent_ || !current_goal_handle_) {
                return;
            }

            if (remaining > 1.5) {
                cancel_sent_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "Remaining angle less than 1.0, cancelling goal...");

                action_client_->async_cancel_goal(current_goal_handle_);
            }

        }

        void result_callback(const GoalHandleMovement::WrappedResult & result){
            RCLCPP_INFO(this->get_logger(),"Result status=%d, delta=%f",result.code,result.result->final_position[0]);

            //rclcpp::shutdown();
        }

        //CLIENT
        rclcpp_action::Client<Movement>::SharedPtr action_client_;

        //VARIABLES
        GoalHandleMovement::SharedPtr current_goal_handle_;
        bool cancel_sent_;
        double input;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    // Crea il nodo client
    auto node = std::make_shared<RobotActionClient>();

    double user_goal;

    std::cout << "Insert x GOAL: ";
    if (!(std::cin >> user_goal)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    node->send_goal(user_goal);
    
    // Spin per qualche tempo per inviare goal e ricevere feedback
    rclcpp::spin(node);

    //rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "turtlesim_custom_msgs/srv/set_threshold.hpp"
#include <cmath>

using std::placeholders::_1;

class ChangeThreshold: public rclcpp::Node
{
    public:
        ChangeThreshold(): Node("change_threshold_node")
        {
            
            threshold_client_ = this->create_client<turtlesim_custom_msgs::srv::SetThreshold>("set_threshold");

        }
    
        void changeThreshold(float new_threshold)
        {
            auto change_request = std::make_shared<turtlesim_custom_msgs::srv::SetThreshold::Request>();
            change_request->threshold = new_threshold;

            //Wait for the service to be available
            while (!threshold_client_->wait_for_service(std::chrono::seconds(1))) 
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
            }

            //Call the service
            auto threshold_result_future = threshold_client_->async_send_request(change_request);

            //Wait for the result
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), threshold_result_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = threshold_result_future.get();
                RCLCPP_INFO(this->get_logger(), "Threshold changed to '%.2f': %s", new_threshold, result->success ? "SUCCESS" : "FAILED");
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call set_threshold service");
            }
        }

    rclcpp::Client<turtlesim_custom_msgs::srv::SetThreshold>::SharedPtr threshold_client_; 
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChangeThreshold>();

    float user_input;
    
    while (rclcpp::ok()) 
    {
        std::cout << "\nEnter a new threshold value: ";
        
        if (std::cin >> user_input) 
        {
            node->changeThreshold(user_input);
        } 
        else 
        {
            break;
        }
    }
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include "rwa5/service_client.hpp"

// implementation of method competition_state_subscriber_callback
void ServiceClientNode::competition_state_subscriber_callback(ariac_msgs::msg::CompetitionState::SharedPtr msg){
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY){
        m_competition_state = true;
        call_competition_service();
    }
}

// implementation of method call_competition_state
void ServiceClientNode::call_competition_service(){
    
    // Wait for service to become available
    while (!m_competition_start_client->wait_for_service(std::chrono::seconds(1))){
        if  (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        else{
            RCLCPP_INFO_STREAM(this->get_logger(), "Service \'/ariac/start_competition\' is not available, waiting again...");
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Service \'/ariac/start_competition\' is not available, waiting again...");
    }

    // Send request to the server
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto future_result = m_competition_start_client->async_send_request(request, std::bind(&ServiceClientNode::response_callback,
                                                                                    this, std::placeholders::_1));
}

// implementation of method response_callback
void ServiceClientNode::response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
    
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready){
        RCLCPP_INFO_STREAM(this->get_logger(), "Competition service started: " << future.get()->success);
    }
    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Service In-Progress...");
    }
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto competition_state_node = std::make_shared<ServiceClientNode>();
    rclcpp::spin(competition_state_node);
    rclcpp::shutdown();
}
#include <rclcpp/rclcpp.hpp>
#include "rwa67/shipOrder.hpp"

#include <string>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>


using namespace std::chrono_literals;

/**
* @brief  Callback function for the topic /ship_order
* 
* @param msg Integer message containing agv number
*/
shipOrder::shipOrder(const std::string &nodeName) : Node(nodeName){

    //Create subscription
    ship_order_subscriber  = this->create_subscription<std_msgs::msg::UInt8>("/ship_order", 10, 
    std::bind(&shipOrder::shipOrderCallback, this, std::placeholders::_1));
}




/**
* @brief  Callback function for the topic /ship_order
* 
* @param msg Integer message containing agv number
*/
void shipOrder::shipOrderCallback(const std_msgs::msg::UInt8::SharedPtr msg){
    
    RCLCPP_INFO(this->get_logger(), "Recieved Ship request.");
    agv_ship_exec(msg->data);
}


void shipOrder::agv_ship_exec(uint8_t agv_num){
    RCLCPP_INFO(this->get_logger(), "Generating Ship request.");
    
    client_ship_agv = this->create_client<std_srvs::srv::Trigger>("/ariac/agv"+std::to_string(agv_num)+"_lock_tray");
    
    RCLCPP_INFO(this->get_logger(), "Waiting for Ship request Service...");
    // Wait for the service to become available
    while (!client_ship_agv->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "Serice ready, making async ship request.");
    // Create a request and send it to the server
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    switch(agv_num){
        case 1:
        {
            auto future_result = client_ship_agv->async_send_request(request, std::bind(&shipOrder::agv1_lock_callback, this, std::placeholders::_1));
            break;
        }

        case 2:
        {
            auto future_result = client_ship_agv->async_send_request(request, std::bind(&shipOrder::agv2_lock_callback, this, std::placeholders::_1));
            break;
        }

        case 3:
        {
            auto future_result = client_ship_agv->async_send_request(request, std::bind(&shipOrder::agv3_lock_callback, this, std::placeholders::_1));
            break;
        }

        case 4:
        {
            auto future_result = client_ship_agv->async_send_request(request, std::bind(&shipOrder::agv4_lock_callback, this, std::placeholders::_1));
            break;
        }
    }
}

void shipOrder::agv_move_exec(uint8_t agv_num){
    client_move_agv = this->create_client<ariac_msgs::srv::MoveAGV>("/ariac/move_agv"+std::to_string(agv_num));
    
    // Wait for the service to become available
    while (!client_move_agv->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request and send it to the server
    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = request->WAREHOUSE;

    switch(agv_num){
        case 1:
        {
            auto future_result = client_move_agv->async_send_request(request, std::bind(&shipOrder::agv1_move_callback, this, std::placeholders::_1));
            break;
        }

        case 2:
        {
            auto future_result = client_move_agv->async_send_request(request, std::bind(&shipOrder::agv2_move_callback, this, std::placeholders::_1));
            break;
        }

        case 3:
        {
            auto future_result = client_move_agv->async_send_request(request, std::bind(&shipOrder::agv3_move_callback, this, std::placeholders::_1));
            break;
        }

        case 4:
        {
            auto future_result = client_move_agv->async_send_request(request, std::bind(&shipOrder::agv4_move_callback, this, std::placeholders::_1));
            break;
        }

    }
    
}


void shipOrder::agv4_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Move AGV Result: %s!", success_flg.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void shipOrder::agv4_lock_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Tray Lock Result: %s!", success_flg.c_str());
        if (success == true)
        {
            agv_move_exec(4);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "AGV Tray Lock Failed!");
        }
        
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}


void shipOrder::agv3_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Move AGV Result: %s!", success_flg.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void shipOrder::agv3_lock_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Tray Lock Result: %s!", success_flg.c_str());
        if (success == true)
        {
            agv_move_exec(3);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "AGV Tray Lock Failed!");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}


void shipOrder::agv2_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Move AGV Result: %s!", success_flg.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void shipOrder::agv2_lock_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Tray Lock Result: %s!", success_flg.c_str());
        if (success == true)
        {
            agv_move_exec(2);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "AGV Tray Lock Failed!");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}


void shipOrder::agv1_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Move AGV Result: %s!", success_flg.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void shipOrder::agv1_lock_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle){
    auto status = wait_handle.wait_for(1s);
    if (status == std::future_status::ready)
    {
        bool success = wait_handle.get()->success;
        std::string success_flg;
        if (success == true)
        {
            success_flg = "Success";
        }
        else 
        {
            success_flg = "Failed";
        }
        
        RCLCPP_INFO(this->get_logger(), "Tray Lock Result: %s!", success_flg.c_str());
        if (success == true)
        {
            agv_move_exec(1);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "AGV Tray Lock Failed!");
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}



/**
  * @brief Main Function for shipOrder executable
  * 
  * Initializes ROS and shipOrder node
  *
  * @param argc argument count
  * @param argv argumant list
  * @return int 
  */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<shipOrder>("shipOrder");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
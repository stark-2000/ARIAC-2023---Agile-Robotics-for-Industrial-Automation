#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
/**
 * @brief Class declaration for shipping ARIAC orders
 * 
 */
class shipOrder: public rclcpp::Node
{
    public:
        /**
        * @brief Construct a new ship Order object
        * 
        * @param nodeName 
        */
        shipOrder(const std::string &nodeName);
        void agv_ship_exec(uint8_t agv_num);
        void agv_move_exec(uint8_t agv_num);

    private:
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ship_order_subscriber; /*!< Varaible to hold the ARIAC Order subscription handle*/

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_ship_agv;
        rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client_move_agv;


        void agv4_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle);
        void agv4_ship_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle);

        void agv3_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle);
        void agv3_ship_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle);

        void agv2_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle);
        void agv2_ship_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle);
        
        void agv1_move_callback(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture wait_handle);
        void agv1_ship_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle);

        void start_ship_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture wait_handle);

        /**
        * @brief  Callback function for the topic /ship_order
        * 
        * @param msg Integer message containing agv number
        */
        void shipOrderCallback(const std_msgs::msg::Int8::SharedPtr msg);
};  
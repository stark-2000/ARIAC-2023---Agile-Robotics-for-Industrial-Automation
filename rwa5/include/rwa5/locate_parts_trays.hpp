#ifndef __LOCATE_PARTS_TRAYS_HPP
#define __LOCATE_PARTS_TRAYS_HPP

/**
 * @file locate_parts_trays.hpp
 * @author Shreejay Badshah (sbadshah@umd.edu)
 * @brief Functions to locate and transform the poses of parts and trays in ARIAC competition based on camera data
 * @version 1.0
 * @date 2023-07-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <memory>
#include <geometry_msgs/msg/pose.hpp>

#include "rwa5/ariac_constants.hpp"
#include "rwa5/ariac_tf_util.hpp"

/**
 * @brief This class locates parts and trays based on AL camera feedback and publishes transformed pose for the order manager 
 * 
 */
class LocatePartsTraysNode : public rclcpp::Node {

public:
    // default ctor
    LocatePartsTraysNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing parts and tray poses in the world frame.");

        // initializing the camera image data message for the left bin
        m_left_bin_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

        // initializing the camera image data message for the right bin
        m_right_bin_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

        // initializing the camera image data message for kitting tray1
        m_kitting_tray1_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

        // initializing the camera image data message for kitting tray2
        m_kitting_tray2_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

        // initializing the bool value to check if the left bin camera subscriber has received the message
        m_left_bin_camera_data_bool = true;

        // initializing the bool value to check if the right bin camera subscriber has received the message
        m_right_bin_camera_data_bool = true;

        // initializing the bool value to check if the kitting tray1 camera subscriber has received the message
        m_kitting_tray1_camera_data_bool = true;

        // initializing the bool value to check if the kitting tray2 camera subscriber has received the message
        m_kitting_tray2_camera_data_bool = true;

        // ariac trainsform utility for finding pose in world frame
        m_ariac_tf_util = std::make_shared<ariac_tf_util>();

        // subscriber for AdvancedLogicalCameraImage of left bin camera data
        m_left_bin_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                                                              std::bind(&LocatePartsTraysNode::left_bin_camera_data_subscriber_callback,
                                                                                                                        this, std::placeholders::_1));

        // subscriber for AdvancedLogicalCameraImage of right bin camera data
        m_right_bin_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
                                                                                                               std::bind(&LocatePartsTraysNode::right_bin_camera_data_subscriber_callback,
                                                                                                                         this, std::placeholders::_1));

        // subscriber for AdvancedLogicalCameraImage of kitting tray1 camera data
        m_kitting_tray1_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
                                                                                                                   std::bind(&LocatePartsTraysNode::kitting_tray1_camera_data_subscriber_callback,
                                                                                                                             this, std::placeholders::_1));

        // subscriber for AdvancedLogicalCameraImage of kitting tray2 camera data
        m_kitting_tray2_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
                                                                                                                   std::bind(&LocatePartsTraysNode::kitting_tray2_camera_data_subscriber_callback,
                                                                                                                             this, std::placeholders::_1));
        
        // publisher of left bin camera data
        m_left_bin_camera_publisher = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>("/left_bin_camera_data", rclcpp::SensorDataQoS());
        
        // publisher of right bin camera data
        m_right_bin_camera_publisher = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>("/right_bin_camera_data", rclcpp::SensorDataQoS());                          
        
        // publisher of kitting tray1 camera data
        m_kitting_tray1_camera_publisher = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>("/kitting_tray1_camera_data", rclcpp::SensorDataQoS());
        
        // publisher of kitting tray2 camera data
        m_kitting_tray2_camera_publisher = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>("/kitting_tray2_camera_data", rclcpp::SensorDataQoS());

    }

private:

    // attributes
    /**
     * @brief Object of AdvancedLogicalCamera for the left bin
     *
     */
    ariac_msgs::msg::AdvancedLogicalCameraImage m_left_bin_camera;

    /**
     * @brief Object of AdvancedLogicalCamera for the right bin
     *
     */
    ariac_msgs::msg::AdvancedLogicalCameraImage m_right_bin_camera;

    /**
     * @brief Object of AdvancedLogicalCamera for the kitting tray1
     *
     */
    ariac_msgs::msg::AdvancedLogicalCameraImage m_kitting_tray1_camera;

    /**
     * @brief Object of AdvancedLogicalCamera for the kitting tray2
     *
     */
    ariac_msgs::msg::AdvancedLogicalCameraImage m_kitting_tray2_camera;

    /**
     * @brief Shared pointer to create_subscription object of Order
     *
     */
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr m_order_subscriber;

    /**
     * @brief Shared pointer to create_subscription object of AdvancedLogicalCameraImage for the left bin
     *
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_left_bin_camera_subscriber;

    /**
     * @brief Shared pointer to create_subscription object of AdvancedLogicalCameraImage for the right bin
     *
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_right_bin_camera_subscriber;

    /**
     * @brief Shared pointer to create_subscription object of AdvancedLogicalCameraImage for the kitting tray1
     *
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_kitting_tray1_camera_subscriber;

    /**
     * @brief Shared pointer to create_subscription object of AdvancedLogicalCameraImage for the kitting tray2
     *
     */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_kitting_tray2_camera_subscriber;

    /**
     * @brief Bool value to check if the left bin camera subscriber has received the message
     * 
     */
    bool m_left_bin_camera_data_bool;

    /**
     * @brief Bool value to check if the right bin camera subscriber has received the message
     * 
     */
    bool m_right_bin_camera_data_bool;

    /**
     * @brief Bool value to check if the kitting tray1 camera subscriber has received the message
     * 
     */
    bool m_kitting_tray1_camera_data_bool;

    /**
     * @brief Bool value to check if the kitting tray2 camera subscriber has received the message
     * 
     */
    bool m_kitting_tray2_camera_data_bool;

    /**
     * @brief Shared pointer to create_publisher object of AdvancedLogicalCameraImage for the left bin camera data
     * 
     */
    rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_left_bin_camera_publisher;

    /**
     * @brief Shared pointer to create_publisher object of AdvancedLogicalCameraImage for the right bin camera data
     * 
     */
    rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_right_bin_camera_publisher;

    /**
     * @brief Shared pointer to create_publisher object of AdvancedLogicalCameraImage for the kitting tray1 camera data
     * 
     */
    rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_kitting_tray1_camera_publisher;

    /**
     * @brief Shared pointer to create_publisher object of AdvancedLogicalCameraImage for the kitting tray2 camera data
     * 
     */
    rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr m_kitting_tray2_camera_publisher;

    /**
     * @brief Shared pointer to ariac_tf_util
     *
     */
    std::shared_ptr<ariac_tf_util> m_ariac_tf_util;

    // methods
    /**
     * @brief Subscriber to AdvancedLogicalCameraImage for the left bin
     *
     * @param msg Shared pointer to msg object
     */
    void left_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscriber to AdvancedLogicalCameraImage for the right bin
     *
     * @param msg Shared pointer to msg object
     */
    void right_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscriber to AdvancedLogicalCameraImage for the kitting tray1
     *
     * @param msg Shared pointer to msg object
     */
    void kitting_tray1_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscriber to AdvancedLogicalCameraImage for the kitting tray2
     *
     * @param msg Shared pointer to msg object
     */
    void kitting_tray2_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Convert uint8_t to std::string to print on terminal
     *
     * @param literal_part_type of type uint8_t
     * @return std::string
     */
    std::string part_type(uint8_t literal_part_type);

}; // class LocatePartsTraysNode

#endif
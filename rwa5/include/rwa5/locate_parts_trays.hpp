#ifndef __LOCATE_PARTS_TRAYS_HPP
#define __LOCATE_PARTS_TRAYS_HPP

#include <rclcpp/rclcpp.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <memory>

#include "rwa5/ariac_constants.hpp"
#include "rwa5/ariac_tf_util.hpp"


class LocatePartsTraysNode : public rclcpp::Node {

    public:

     // default ctor
     LocatePartsTraysNode(std::string node_name): Node(node_name) {

     // initializing the camera image data message for the left bin
     m_left_bin_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

     // initializing the camera image data message for the right bin
     m_right_bin_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

     // initializing the camera image data message for kitting tray1
     m_kitting_tray1_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

     // initializing the camera image data message for kitting tray2
     m_kitting_tray2_camera = ariac_msgs::msg::AdvancedLogicalCameraImage();

    auto node_ptr = shared_from_this();
     _ariac_tf_util = std::make_shared<ariac_tf_util>(node_ptr);

     // subscriber callback for Order
     m_order_subscriber = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10,
                                                                  std::bind(&LocatePartsTraysNode::receive_order, 
                                                                  this, std::placeholders::_1));
     
     // subscriber callback for AdvancedLogicalCameraImage of left bin
     m_left_bin_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/left_bins_camera/image", 
                                                                                                           rclcpp::SensorDataQoS(),
                                                                                                           std::bind(&LocatePartsTraysNode::left_bin_camera_data_subscriber_callback, 
                                                                                                           this, std::placeholders::_1));

     // subscriber callback for AdvancedLogicalCameraImage of right bin
     m_right_bin_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/right_bins_camera/image", 
                                                                                                           rclcpp::SensorDataQoS(),
                                                                                                           std::bind(&LocatePartsTraysNode::right_bin_camera_data_subscriber_callback, 
                                                                                                           this, std::placeholders::_1));
     
     // subscriber callback for AdvancedLogicalCameraImage of kitting tray1 
     m_kitting_tray1_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts1_camera/image", 
                                                                                                           rclcpp::SensorDataQoS(),
                                                                                                           std::bind(&LocatePartsTraysNode::kitting_tray1_camera_data_subscriber_callback, 
                                                                                                           this, std::placeholders::_1));
     
     // subscriber callback for AdvancedLogicalCameraImage of kitting tray2
     m_kitting_tray2_camera_subscriber = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>("/ariac/sensors/kts2_camera/image", 
                                                                                                           rclcpp::SensorDataQoS(),
                                                                                                           std::bind(&LocatePartsTraysNode::kitting_tray2_camera_data_subscriber_callback, 
                                                                                                           this, std::placeholders::_1));

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
      * @brief Shared pointer to create_timer object for the left bin
      * 
      */
     rclcpp::TimerBase::SharedPtr m_left_bin_timer;

     /**
      * @brief Shared pointer to create_timer object for the right bin
      * 
      */
     rclcpp::TimerBase::SharedPtr m_right_bin_timer;

     /**
      * @brief Shared pointer to create_timer object for the kitting tray1
      * 
      */
     rclcpp::TimerBase::SharedPtr m_kitting_tray1_timer;

     /**
      * @brief Shared pointer to create_timer object for the kitting tray2
      * 
      */
     rclcpp::TimerBase::SharedPtr m_kitting_tray2_timer;
     
     /**
      * @brief 
      * 
      */
     std::shared_ptr<ariac_tf_util> _ariac_tf_util;
     
     
     // methods
     /**
      * @brief Subscriber to Order
      * 
      * @param msg Shared pointer to msg object
      */
     void receive_order(ariac_msgs::msg::Order::SharedPtr msg);

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

}; // class LocatePartsTrays

#endif
#include <rclcpp/rclcpp.hpp>
#include "rwa5/locate_parts_trays.hpp"
#include "rwa5/ariac_constants.hpp"
#include "rwa5/ariac_tf_util.hpp"

#include <string>

// implementation of method left_bin_camera_data_subscriber_callback
void LocatePartsTraysNode::left_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_left_bin_camera.part_poses = std::vector<ariac_msgs::msg::PartPose>(msg->part_poses.size());
    
    if (m_left_bin_camera_data_bool){
        for (long unsigned int i{0}; i < msg->part_poses.size(); i++){
            
            m_left_bin_camera.part_poses.at(i).part.color = msg->part_poses.at(i).part.color;
            m_left_bin_camera.part_poses.at(i).part.type = msg->part_poses.at(i).part.type;
            m_left_bin_camera.part_poses.at(i).pose = m_ariac_tf_util->get_object_pose_world(shared_from_this(), ARIAC_FRAME::L_BIN_CAMERA_FRAME, msg->part_poses.at(i).pose);
        }

        // publish camera data
        m_left_bin_camera_publisher->publish(m_left_bin_camera);
        m_left_bin_camera_data_bool = false;
    }
}

// implementation of method right_bin_camera_data_subscriber_callback
void LocatePartsTraysNode::right_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_right_bin_camera.part_poses = std::vector<ariac_msgs::msg::PartPose>(msg->part_poses.size());
    
    if (m_right_bin_camera_data_bool){
        for (long unsigned int i{0}; i < msg->part_poses.size(); i++){
            
            m_right_bin_camera.part_poses.at(i).part.color = msg->part_poses.at(i).part.color;
            m_right_bin_camera.part_poses.at(i).part.type = msg->part_poses.at(i).part.type;
            m_right_bin_camera.part_poses.at(i).pose = m_ariac_tf_util->get_object_pose_world(shared_from_this(), ARIAC_FRAME::R_BIN_CAMERA_FRAME, msg->part_poses.at(i).pose);
        }

        // publish camera data
        m_right_bin_camera_publisher->publish(m_right_bin_camera);
        m_right_bin_camera_data_bool = false;

    }
}

// implementation of method kitting_tray1_camera_data_subscriber_callback
void LocatePartsTraysNode::kitting_tray1_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_kitting_tray1_camera.tray_poses = std::vector<ariac_msgs::msg::KitTrayPose>(msg->tray_poses.size());
    
    if (m_kitting_tray1_camera_data_bool){
        for (long unsigned int i{0}; i < msg->tray_poses.size(); i++){
            
            m_kitting_tray1_camera.tray_poses.at(i).id = msg->tray_poses.at(i).id;
            m_kitting_tray1_camera.tray_poses.at(i).pose = m_ariac_tf_util->get_object_pose_world(shared_from_this(), ARIAC_FRAME::KTS1_BIN_CAMERA_FRAME, msg->tray_poses.at(i).pose);
        }

        // publish camera data
        m_kitting_tray1_camera_publisher->publish(m_kitting_tray1_camera);
        m_kitting_tray1_camera_data_bool = false;
    }
}

// implementation of method kitting_tray2_camera_data_subscriber_callback
void LocatePartsTraysNode::kitting_tray2_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
 
    m_kitting_tray2_camera.tray_poses = std::vector<ariac_msgs::msg::KitTrayPose>(msg->tray_poses.size());

    if (m_kitting_tray2_camera_data_bool){
        for (long unsigned int i{0}; i < msg->tray_poses.size(); i++){
            
            m_kitting_tray2_camera.tray_poses.at(i).id = msg->tray_poses.at(i).id;
            m_kitting_tray2_camera.tray_poses.at(i).pose = m_ariac_tf_util->get_object_pose_world(shared_from_this(), ARIAC_FRAME::KTS2_BIN_CAMERA_FRAME, msg->tray_poses.at(i).pose);
        }

        // publish camera data
        m_kitting_tray2_camera_publisher->publish(m_kitting_tray2_camera);
        m_kitting_tray2_camera_data_bool = false;
    }
}

// implementation of method part_type
std::string LocatePartsTraysNode::part_type(uint8_t literal_part_type){
    switch (literal_part_type)
    {
    case 0:
        return std::string{"Red"};
    
    case 1:
        return std::string{"Green"};

    case 2:
        return std::string{"Blue"};

    case 3:
        return std::string{"Orange"};

    case 4:
        return std::string{"Purple"};
    
    case 10:
        return std::string{"Battery"};

    case 11:
        return std::string{"Pump"};

    case 12:
        return std::string{"Sensor"};
    
    case 13:
        return std::string{"Regulator"};
    
    default:
        return std::string{"default part"};
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto locate_parts_trays_node = std::make_shared<LocatePartsTraysNode>("locate_parts_trays");
    rclcpp::spin(locate_parts_trays_node);
    rclcpp::shutdown();
}

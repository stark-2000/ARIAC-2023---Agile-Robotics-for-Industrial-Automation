#include <rclcpp/rclcpp.hpp>
#include "rwa5/locate_parts_trays.hpp"
#include "rwa5/ariac_constants.hpp"
#include "rwa5/ariac_tf_util.hpp"

// implementation of method receive_order
void LocatePartsTraysNode::receive_order(ariac_msgs::msg::Order::SharedPtr msg){
    RCLCPP_INFO_STREAM(this->get_logger(), "Part1 of Order id " << msg->id << ": " << part_type(msg->kitting_task.parts.at(0).part.color) + " " +
                                                                                      part_type(msg->kitting_task.parts.at(0).part.type));

    RCLCPP_INFO_STREAM(this->get_logger(), "Part2 of Order id " << msg->id << ": " << part_type(msg->kitting_task.parts.at(1).part.color) + " " +
                                                                                      part_type(msg->kitting_task.parts.at(1).part.type));

}

// implementation of method left_bin_camera_data_subscriber_callback
void LocatePartsTraysNode::left_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_left_bin_camera.part_poses = std::vector<ariac_msgs::msg::PartPose>(4);
    
    for (long unsigned int i{0}; i < msg->part_poses.size(); i++){
        m_left_bin_camera.part_poses.at(i).part.color = msg->part_poses.at(i).part.color;
        m_left_bin_camera.part_poses.at(i).part.type = msg->part_poses.at(i).part.type;
        m_left_bin_camera.part_poses.at(i).pose.position.x = msg->part_poses.at(i).pose.position.x;
        m_left_bin_camera.part_poses.at(i).pose.position.y = msg->part_poses.at(i).pose.position.y;
        m_left_bin_camera.part_poses.at(i).pose.position.z = msg->part_poses.at(i).pose.position.z;
        m_left_bin_camera.part_poses.at(i).pose.orientation.w = msg->part_poses.at(i).pose.orientation.w;
        m_left_bin_camera.part_poses.at(i).pose.orientation.x = msg->part_poses.at(i).pose.orientation.x;
        m_left_bin_camera.part_poses.at(i).pose.orientation.y = msg->part_poses.at(i).pose.orientation.y;
        m_left_bin_camera.part_poses.at(i).pose.orientation.z = msg->part_poses.at(i).pose.orientation.z;      

       
        m_left_bin_camera.part_poses.at(i).pose = _ariac_tf_util->get_object_pose_world(ARIAC_FRAME::L_BIN_CAMERA_FRAME, msg->part_poses.at(i).pose);
    }  
}

// implementation of method right_bin_camera_data_subscriber_callback
void LocatePartsTraysNode::right_bin_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_right_bin_camera.part_poses = std::vector<ariac_msgs::msg::PartPose>(4);
    
    for (long unsigned int i{0}; i < msg->part_poses.size(); i++){
        m_right_bin_camera.part_poses.at(i).part.color = msg->part_poses.at(i).part.color;
        m_right_bin_camera.part_poses.at(i).part.type = msg->part_poses.at(i).part.type;
        m_right_bin_camera.part_poses.at(i).pose.position.x = msg->part_poses.at(i).pose.position.x;
        m_right_bin_camera.part_poses.at(i).pose.position.y = msg->part_poses.at(i).pose.position.y;
        m_right_bin_camera.part_poses.at(i).pose.position.z = msg->part_poses.at(i).pose.position.z;
        m_right_bin_camera.part_poses.at(i).pose.orientation.w = msg->part_poses.at(i).pose.orientation.w;
        m_right_bin_camera.part_poses.at(i).pose.orientation.x = msg->part_poses.at(i).pose.orientation.x;
        m_right_bin_camera.part_poses.at(i).pose.orientation.y = msg->part_poses.at(i).pose.orientation.y;
        m_right_bin_camera.part_poses.at(i).pose.orientation.z = msg->part_poses.at(i).pose.orientation.z;
        
    }  
}

// implementation of method kitting_tray1_camera_data_subscriber_callback
void LocatePartsTraysNode::kitting_tray1_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

    m_kitting_tray1_camera.tray_poses = std::vector<ariac_msgs::msg::KitTrayPose>(4);
    
    for (long unsigned int i{0}; i < msg->tray_poses.size(); i++){
        m_kitting_tray1_camera.tray_poses.at(i).pose.position.x = msg->tray_poses.at(i).pose.position.x;
        m_kitting_tray1_camera.tray_poses.at(i).pose.position.y = msg->tray_poses.at(i).pose.position.y;
        m_kitting_tray1_camera.tray_poses.at(i).pose.position.z = msg->tray_poses.at(i).pose.position.z;
        m_kitting_tray1_camera.tray_poses.at(i).pose.orientation.w = msg->tray_poses.at(i).pose.orientation.w;
        m_kitting_tray1_camera.tray_poses.at(i).pose.orientation.x = msg->tray_poses.at(i).pose.orientation.x;
        m_kitting_tray1_camera.tray_poses.at(i).pose.orientation.y = msg->tray_poses.at(i).pose.orientation.y;
        m_kitting_tray1_camera.tray_poses.at(i).pose.orientation.z = msg->tray_poses.at(i).pose.orientation.z;
        
    }  
}

// implementation of method kitting_tray2_camera_data_subscriber_callback
void LocatePartsTraysNode::kitting_tray2_camera_data_subscriber_callback(ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){

     m_kitting_tray2_camera.tray_poses = std::vector<ariac_msgs::msg::KitTrayPose>(4);

    for (long unsigned int i{0}; i < msg->tray_poses.size(); i++){
        m_kitting_tray2_camera.tray_poses.at(i).pose.position.x = msg->tray_poses.at(i).pose.position.x;
        m_kitting_tray2_camera.tray_poses.at(i).pose.position.y = msg->tray_poses.at(i).pose.position.y;
        m_kitting_tray2_camera.tray_poses.at(i).pose.position.z = msg->tray_poses.at(i).pose.position.z;
        m_kitting_tray2_camera.tray_poses.at(i).pose.orientation.w = msg->tray_poses.at(i).pose.orientation.w;
        m_kitting_tray2_camera.tray_poses.at(i).pose.orientation.x = msg->tray_poses.at(i).pose.orientation.x;
        m_kitting_tray2_camera.tray_poses.at(i).pose.orientation.y = msg->tray_poses.at(i).pose.orientation.y;
        m_kitting_tray2_camera.tray_poses.at(i).pose.orientation.z = msg->tray_poses.at(i).pose.orientation.z;
        
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
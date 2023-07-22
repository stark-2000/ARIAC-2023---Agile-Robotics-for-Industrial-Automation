#include <chrono>

#include <tf2_kdl/tf2_kdl.h>

#include "rwa5/ariac_tf_util.hpp"

using namespace std::chrono_literals;

/**
 * @brief Construct a new ariac tf util::ariac tf util object
 * 
 * @param _node node reference
 */
ariac_tf_util::ariac_tf_util(){
}


/*
 * Fetches Pose between given two frames from ROS2 tf tree
 */
geometry_msgs::msg::Pose ariac_tf_util::lookup_transform(rclcpp::Node::SharedPtr _node, const ARIAC_FRAME::NAME &source_frame, const ARIAC_FRAME::NAME &target_frame){

        //Check if the frame already avaialble in cache
        if (static_tf_to_world_cache.find(source_frame) == static_tf_to_world_cache.end()) {
            
            geometry_msgs::msg::TransformStamped t_stamped;
            geometry_msgs::msg::Pose pose_out;

            //Look up the TF tree
            try
            {
                t_stamped = tf_buffer->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR_STREAM(_node->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
                return pose_out;
            }

            //Convert to Pose
            pose_out.position.x = t_stamped.transform.translation.x;
            pose_out.position.y = t_stamped.transform.translation.y;
            pose_out.position.z = t_stamped.transform.translation.z;
            pose_out.orientation = t_stamped.transform.rotation;

            //Store tf in static data cache
            static_tf_to_world_cache.insert({source_frame, pose_out});

            return pose_out;

    } else {
        //retrieve TF from the static cache
        geometry_msgs::msg::Pose pose_out= static_tf_to_world_cache[source_frame];
        return pose_out;
    }
}



/*
 * Computes given objects pose in world frame
 */
geometry_msgs::msg::Pose ariac_tf_util::get_object_pose_world(rclcpp::Node::SharedPtr _node, const ARIAC_FRAME::NAME &source_frame, geometry_msgs::msg::Pose pose){

    geometry_msgs::msg::Pose camera_pose_in_world = this->lookup_transform(_node, source_frame, ARIAC_FRAME::WORLD);
    geometry_msgs::msg::Pose object_pose_in_camera = pose;

    //Multiply frames to get the objects pose in world frame
    geometry_msgs::msg::Pose object_pose_in_world = multiply_kdl_frames(camera_pose_in_world, object_pose_in_camera);

    return object_pose_in_world;
}



/*
 * Multiplies two poses using kdl library
 */
geometry_msgs::msg::Pose ariac_tf_util::multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
{
    KDL::Frame frame1;
    KDL::Frame frame2;

    tf2::fromMsg(pose1, frame1);
    tf2::fromMsg(pose2, frame2);
    
    KDL::Frame frame3 = frame1 * frame2;

    return tf2::toMsg(frame3);
}
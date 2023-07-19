#pragma once

#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include "ariac_constants.cpp"


/**
 * @brief This is a helper class to convert pose information from one frame to another
 * 
 */
class ariac_tf_util{

    public:
        tf_util(rclcpp::Node& _node);


        /**
        * @brief Convert Pose to World Frame
        *
        * @param source_frame Source Frame of the Pose
        * @param pose Pose of the object
        */
        geometry_msgs::msg::Pose get_object_pose_world(const ARIAC_FRAME::NAME &source_frame, geometry_msgs::msg::Pose pose);


    private:
        rclcpp::Node node; /*!<a reference to the parent ros node object */

        std::map<ARIAC_FRAME::NAME, geometry_msgs::msg::Pose> static_tf_to_world_cache; //!< Cache Dictionary to store retrieved static TF data
                                                                                 //!< Used to reduce TF look up time for static frames

        /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr}; /*!<Transform listener object  */

        /**
        * @brief Lookup transform between two frames
        *
        * @param ARIAC_FRAME::NAME source_frame - Source frame (child frame) of the transform
        * @param ARIAC_FRAME::NAME target_frame - Target frame (parent frame) of the transform
        * @return geometry_msgs::msg::Pose 
        */
        geometry_msgs::msg::Pose lookup_transform(const ARIAC_FRAME::NAME &source_frame, const ARIAC_FRAME::NAME &target_frame);

        /**
        * @brief Finds pose of object in target frame
        *
        * @param geometry_msgs::msg::Pose pose1 - Pose of object in source frame
        * @param geometry_msgs::msg::Pose pose2 - Pose of source frame in target frame
        * @return geometry_msgs::msg::Pose 
        */
        geometry_msgs::msg::Pose multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);

}
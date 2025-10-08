#include "cw2_grasp.h"
#include "cw2_movement.h"
#include <pcl/common/common.h>

/**
 * @file cw2_grasp.cpp
 * @brief Functions for robot gripper operations and grasp pose calculation.
 *
 * This file provides functionality for gripper control (open, close, and release) as well as
 * sophisticated grasp pose computation for different object shapes (crosses and noughts) in a 3D environment.
 */

namespace cw2_grasp
{

    // Open the Gripper
    bool prepareGripperOpen(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_open_)
    {
        ROS_INFO("prepareGripperOpen: Opening gripper...");
        double width = gripper_open_;
        double eachJoint = width / 2.0;

        std::vector<double> gripperJointTargets(2, eachJoint);
        hand_group_.setJointValueTarget(gripperJointTargets);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("prepareGripperOpen: Planning failed.");
            return false;
        }
        hand_group_.move();
        ros::Duration(0.5).sleep();
        ROS_INFO("prepareGripperOpen: Gripper opened.");
        return true;
    }

    // Close the Gripper
    bool closeGripperForGrasp(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_closed_)
    {
        ROS_INFO("closeGripperForGrasp: Closing gripper to grasp object...");
        double width = gripper_closed_; // Make sure gripper_closed_ is defined in your class (e.g., 0.02)
        double eachJoint = width / 2.0;

        std::vector<double> gripperJointTargets(2, eachJoint);
        hand_group_.setJointValueTarget(gripperJointTargets);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("closeGripperForGrasp: Planning failed.");
            return false;
        }
        hand_group_.move();
        ros::Duration(0.5).sleep();
        ROS_INFO("closeGripperForGrasp: Gripper closed.");
        return true;
    }

    // Release the Gripper
    bool releaseObject(
        moveit::planning_interface::MoveGroupInterface &hand_group_,
        double gripper_open_)
    {
        ROS_INFO("releaseObject: Releasing object by opening gripper...");
        double width = gripper_open_; // Use the same gripper_open_ value as in prepareGripperOpen()
        double eachJoint = width / 2.0;

        std::vector<double> gripperJointTargets(2, eachJoint);
        hand_group_.setJointValueTarget(gripperJointTargets);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("releaseObject: Planning failed.");
            return false;
        }
        hand_group_.move();
        ros::Duration(0.5).sleep();
        ROS_INFO("releaseObject: Object released.");
        return true;
    }

    // Compute Grasp Pose 
    geometry_msgs::PoseStamped computeGraspPose(
        const cw2::ObjectInfo &object,
        double lateral_offset)
    {
        // Start with the object's pose
        geometry_msgs::PoseStamped grasp_pose = object.pose;
        std::string shape_type = object.shapeType;
        double orientation_deg = object.orientation;

        // Convert orientation to radians
        double orientation_rad = orientation_deg * M_PI / 180.0;

        // Calculate grasp orientation based on shape type
        double grasp_yaw = orientation_rad;

        if (shape_type == "cross")
        {
            // For crosses, we need to align with an arm and choose the orientation away from the origin

            // First, calculate vector from origin to object center
            double dir_x = object.pose.pose.position.x; // Vector from origins to object
            double dir_y = object.pose.pose.position.y;
            double dir_angle = std::atan2(dir_y, dir_x); // Angle of this vector

            // Calculate the four possible arm angles (45° offset from main orientation)
            // Each cross has 4 arms at 90° intervals
            double arm_angles[4];
            arm_angles[0] = orientation_rad + M_PI / 4;   // First arm
            arm_angles[1] = arm_angles[0] + M_PI / 2;     // Second arm (90° from first)
            arm_angles[2] = arm_angles[0] + M_PI;         // Third arm (180° from first)
            arm_angles[3] = arm_angles[0] + 3 * M_PI / 2; // Fourth arm (270° from first)

            // Normalize all angles to [0, 2π)
            for (int i = 0; i < 4; i++)
            {
                while (arm_angles[i] < 0)
                    arm_angles[i] += 2 * M_PI;
                while (arm_angles[i] >= 2 * M_PI)
                    arm_angles[i] -= 2 * M_PI;
            }

            // Find the arm angle closest to the direction away from origin
            double target_angle = dir_angle; // We want the arm pointing away from origin
            if (target_angle < 0)
                target_angle += 2 * M_PI; // Normalize to [0, 2π)

            int best_arm = 0;
            double min_angle_diff = 2 * M_PI;

            for (int i = 0; i < 4; i++)
            {
                double angle_diff = std::abs(arm_angles[i] - target_angle);
                // Handle wrap-around case
                if (angle_diff > M_PI)
                    angle_diff = 2 * M_PI - angle_diff;

                if (angle_diff < min_angle_diff)
                {
                    min_angle_diff = angle_diff;
                    best_arm = i;
                }
            }

            // Use the selected arm's orientation
            grasp_yaw = arm_angles[best_arm] + M_PI;

            // Compensate for gripper's 45° offset
            grasp_yaw -= M_PI / 4;

            // Get cluster size information
            size_t point_count = object.cluster->size();

            // Calculate physical dimensions of the cross
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*(object.cluster), min_pt, max_pt);
            double width = max_pt[0] - min_pt[0];
            double height = max_pt[1] - min_pt[1];
            double max_dimension = std::max(width, height);

            // Dynamically scale the lateral offset based on cross size
            // Use both point count and physical dimensions for robustness
            double dynamic_lateral_offset;

            if (point_count > 600 || max_dimension > 0.18)
            {
                // Large cross
                dynamic_lateral_offset = 0.05;
                ROS_INFO("Detected large cross (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count, max_dimension, dynamic_lateral_offset);
            }
            else if (point_count > 350 || max_dimension > 0.12)
            {
                // Medium cross
                dynamic_lateral_offset = 0.035;
                ROS_INFO("Detected medium cross (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count, max_dimension, dynamic_lateral_offset);
            }
            else
            {
                // Small cross
                dynamic_lateral_offset = 0.02;
                ROS_INFO("Detected small cross (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count, max_dimension, dynamic_lateral_offset);
            }

            // Override the passed-in lateral_offset with our dynamic calculation
            double lateral_offset_cross = dynamic_lateral_offset;

            // Calculate offset perpendicular to grasp direction
            double lateral_offset_angle = grasp_yaw + M_PI / 2; // 90° from grasp direction
            grasp_pose.pose.position.x += lateral_offset_cross * std::cos(lateral_offset_angle);
            grasp_pose.pose.position.y += lateral_offset_cross * std::sin(lateral_offset_angle);

            ROS_INFO("Applied %.3fm lateral offset for better gripper alignment for the cross", lateral_offset);

            // Log which arm we selected
            ROS_INFO("Cross arm selection: choosing arm %d at %.1f degrees",
                     best_arm, grasp_yaw * 180.0 / M_PI);
        }
        else if (shape_type == "nought")
        {
            double grasp_yaw_original = grasp_yaw; // Save the original yaw
          
            // Get cluster size information
            size_t point_count_n = object.cluster->size();

            // Calculate physical dimensions of the cross
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*(object.cluster), min_pt, max_pt);
            double width_n = max_pt[0] - min_pt[0];
            double height_n = max_pt[1] - min_pt[1];
            double max_dimension_n = std::max(width_n, height_n);

            // Dynamically scale the lateral offset based on nought size
            double lateral_offset_nought;

            if (point_count_n > 1000)
            {
                // Large nought
                lateral_offset_nought = 0.07;
                ROS_INFO("Detected large nought (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count_n, max_dimension_n, lateral_offset_nought);
            }
            else if (point_count_n > 600 )
            {
                // Medium cross
                lateral_offset_nought = 0.06;
                ROS_INFO("Detected medium nought (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count_n, max_dimension_n, lateral_offset_nought);
            }
            else
            {
                // Small cross
                lateral_offset_nought = 0.05;
                ROS_INFO("Detected small nought (points: %zu, size: %.3fm), using offset: %.3fm",
                         point_count_n, max_dimension_n, lateral_offset_nought);
            }

            // Calculate offset to move grasp point from nought center to edge center
            grasp_pose.pose.position.x += lateral_offset_nought * std::cos(grasp_yaw_original);
            grasp_pose.pose.position.y += lateral_offset_nought * std::sin(grasp_yaw_original); 
            
            // Compensate for gripper's 45° offset
            grasp_yaw = grasp_yaw + M_PI_4; 
        
            ROS_INFO("Nought grasp angle: %.1f degrees",
                     grasp_yaw * 180.0 / M_PI);
        }

        // Define the end-effector orientation for top-down grasping
        tf2::Quaternion q;
        q.setRPY(0, M_PI, grasp_yaw); // Roll=0, Pitch=π (pointing down), calculated Yaw
        q.normalize();
        grasp_pose.pose.orientation = tf2::toMsg(q);

        // Calculate offset based on object size and shape
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*(object.cluster), min_pt, max_pt);

        double object_size_x = max_pt[0] - min_pt[0];
        double object_size_y = max_pt[1] - min_pt[1];
        double object_size = std::max(object_size_x, object_size_y);

        // This is a repeat of offset calculation for cross
        // It's a bug that needs to be fixed but as of now the task is 
        // tuned to work with this additional offset
        if (shape_type == "cross")
        {
            double offset_factor = 0.3;
            double offset_distance = object_size * offset_factor;

            // Apply offset in the grasp direction
            grasp_pose.pose.position.x += offset_distance * std::cos(grasp_yaw);
            grasp_pose.pose.position.y += offset_distance * std::sin(grasp_yaw);
        }

        // Ensure Z height is appropriate for grasping (slightly above table)
        grasp_pose.pose.position.z = std::max(grasp_pose.pose.position.z, 0.05);

        // Log the grasp pose calculation
        ROS_INFO("computeGraspPose: Calculated grasp for %s at [%.3f, %.3f, %.3f], yaw=%.1f°",
                 shape_type.c_str(),
                 grasp_pose.pose.position.x,
                 grasp_pose.pose.position.y,
                 grasp_pose.pose.position.z,
                 grasp_yaw * 180.0 / M_PI);

        return grasp_pose;
    }

}
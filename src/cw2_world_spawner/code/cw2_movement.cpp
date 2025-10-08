#include "cw2_movement.h"

/**
 * @file cw2_movement.cpp
 * @brief Contains utility functions for robot arm movement and manipulation.
 *
 * This file implements various movement strategies for a robotic arm,
 * including moving to predefined positions, planning and executing
 * Cartesian paths, observing the workspace, and manipulating objects.
 * The functions include robust error handling and recovery strategies
 * for planning and execution failures.
 */

namespace cw2_movement
{
    // Move arm to the ready position with parameterized planning settings
    bool moveToReadyPosition(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        double planning_time,
        double velocity_scaling,
        double accel_scaling,
        double eef_step)
    {
        ROS_INFO("Moving to 'ready' position...");

        // Stop any current movement and clear targets
        arm_group_.stop();
        arm_group_.clearPoseTargets();

        // Apply planning parameters
        arm_group_.setPlanningTime(planning_time);

        // Set the target position
        arm_group_.setNamedTarget("ready");

        // Execute the movement
        moveit::planning_interface::MoveItErrorCode result = arm_group_.move();

        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Successfully moved to 'ready' position");
            return true;
        }
        else
        {
            ROS_WARN("Primary move failed (error code: %d) to ready position. Trying joint-space planning fallback...", result.val);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("Fallback joint-space planning succeeded and moved to 'ready' position");
                    return true;
                }
                else
                {
                    ROS_WARN("Fallback execution failed to 'ready' position");
                }
            }
            else
            {
                ROS_WARN("Fallback planning failed to 'ready' position");
            }
            return false;
        }
    }

    // Plan and execute a Cartesian path to the specified waypoints
    bool planAndExecuteCartesianPath(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        const std::vector<geometry_msgs::Pose> &waypoints,
        const std::string &action_name,
        double eef_step)
    {
        moveit_msgs::RobotTrajectory trajectory;
        int max_planning_attempts = 10;
        double jump_threshold = 0.0; // Deprecated, but required by the current overload.

        // Outer loop: Try planning with increasingly larger step sizes
        for (int attempt = 1; attempt <= max_planning_attempts; attempt++)
        {
            // Compute path and check if we achieved at least 70% of the requested path
            ROS_INFO_STREAM(action_name << ": Planning attempt number " << attempt << " with eef_step = " << eef_step);
            double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction >= 0.7)
            {
                ROS_INFO_STREAM(action_name << ": Cartesian path planned successfully with fraction " << fraction);
                int max_exec_attempts = 10;

                // Inner loop: Try executing the planned path multiple times
                for (int exec = 1; exec <= max_exec_attempts; exec++)
                {
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory;

                    // Attempt execution of the planned path
                    if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    {
                        ROS_INFO_STREAM(action_name << ": Execution succeeded on attempt " << exec);
                        return true;
                    }
                    else
                    {
                        ROS_WARN_STREAM(action_name << ": Execution attempt " << exec << " failed");

                        // If execution fails, try perturbing the final position slightly
                        // This can help overcome local minima in the motion planning
                        if (exec < max_exec_attempts && !waypoints.empty())
                        {
                            geometry_msgs::Pose perturbed = waypoints.back();
                            perturbed.position.x += 0.01;
                            perturbed.position.y += 0.01;
                            perturbed.position.z += 0.01;
                            ROS_INFO_STREAM(action_name << ": Perturbing final waypoint for retry");

                            // Recompute path with perturbed endpoint
                            std::vector<geometry_msgs::Pose> perturbed_waypoints = waypoints;
                            perturbed_waypoints.back() = perturbed;
                            fraction = arm_group_.computeCartesianPath(perturbed_waypoints, eef_step, jump_threshold, trajectory);
                            if (fraction < 0.7)
                                ROS_WARN_STREAM(action_name << ": Perturbed path computation only achieved fraction " << fraction);
                        }
                    }
                }

                // If execution still fails, try relaxing the tolerance parameters adaptively.
                ROS_WARN_STREAM(action_name << ": Execution failed after retries. Relaxing tolerances.");
                double original_pos_tol = arm_group_.getGoalPositionTolerance();
                double original_orient_tol = arm_group_.getGoalOrientationTolerance();

                // Increase tolerances by 25%
                arm_group_.setGoalJointTolerance(original_pos_tol * 1.25);
                arm_group_.setGoalOrientationTolerance(original_orient_tol * 1.25);
                ROS_INFO_STREAM(action_name << ": Relaxed tolerances to pos_tol = "
                                            << arm_group_.getGoalPositionTolerance() << ", orient_tol = "
                                            << arm_group_.getGoalOrientationTolerance());

                // Try one final execution with relaxed tolerances
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO_STREAM(action_name << ": Execution succeeded with relaxed tolerances.");

                    // Restore the original tolerances
                    arm_group_.setGoalJointTolerance(original_pos_tol);
                    arm_group_.setGoalOrientationTolerance(original_orient_tol);

                    return true;
                }
                else
                {
                    ROS_ERROR_STREAM(action_name << ": Execution failed even with relaxed tolerances.");

                    // Restore the original tolerances
                    arm_group_.setGoalJointTolerance(original_pos_tol);
                    arm_group_.setGoalOrientationTolerance(original_orient_tol);
                }
            }
            else
            {
                ROS_WARN_STREAM(action_name << ": Cartesian path planning achieved only " << (fraction * 100.0)
                                            << "%. Retrying with relaxed constraints.");
            }

            // Increase the step size to relax planning constraints.
            eef_step *= 1.2;
        }

        ROS_ERROR_STREAM(action_name << ": All planning attempts failed.");
        return false;
    }

    // Get a set of observation poses for scanning the workspace
    std::vector<geometry_msgs::PoseStamped> getObservationPoses()
    {
        std::vector<geometry_msgs::PoseStamped> poses;

        // Create a pose stamped message template
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "panda_link0"; // Base frame
        pose.header.stamp = ros::Time::now();

        // Add observation positions
        std::vector<std::pair<double, double>> corners = {
            {0.0, 0.4},   // Center-left
            {0.4, 0.4},   // Front-left
            {0.4, 0.0},   // Front-center
            {0.4, -0.4},  // Front-right
            {0.0, -0.4},  // Center-right
            {-0.4, -0.4}, // Back-right
            {-0.4, 0.0},  // Back-center
            {-0.4, 0.4}   // Back-left
        };

        for (const auto &corner : corners)
        {
            pose.pose.position.x = corner.first;
            pose.pose.position.y = corner.second;
            pose.pose.position.z = 0.7;

            // Use downward-looking orientation
            tf2::Quaternion q_down;
            q_down.setRPY(0, M_PI, M_PI_4 + M_PI_2);
            q_down.normalize();
            pose.pose.orientation = tf2::toMsg(q_down);

            poses.push_back(pose);
            ROS_INFO("getObservationPoses: Added observation point [%.2f, %.2f, %.2f], looking downward",
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }

        return poses;
    }

    // Move robot to an observation pose
    bool moveToObservationPose(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &pose,
        double eef_step)
    {
        ROS_INFO("moveToObservationPose: [%.2f, %.2f, %.2f]",
                 pose.pose.position.x,
                 pose.pose.position.y,
                 pose.pose.position.z);

        arm_group.stop();
        arm_group.clearPoseTargets();

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose.pose);

        if (!planAndExecuteCartesianPath(arm_group, waypoints, "moveToObservationPose", eef_step))
        {
            ROS_WARN("moveToObservationPose: Primary Cartesian planning failed. Trying joint-space fallback...");
            arm_group.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                if (arm_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("moveToObservationPose: Fallback joint-space planning succeeded.");
                    return true;
                }
                else
                {
                    ROS_ERROR("moveToObservationPose: Fallback execution failed.");
                    return false;
                }
            }
            else
            {
                ROS_ERROR("moveToObservationPose: Fallback planning failed.");
                return false;
            }
        }

        ROS_INFO("moveToObservationPose: Motion succeeded");
        return true;
    }

    // Move the arm to a specified target pose with a fallback to joint-space planning
    bool moveToTargetPose(
        moveit::planning_interface::MoveGroupInterface &arm_group_,
        const geometry_msgs::PoseStamped &target_pose,
        double eef_step,
        double jump_threshold)
    {
        ROS_INFO("Moving to target pose...");
        arm_group_.stop();
        arm_group_.clearPoseTargets();

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose.pose);

        if (!planAndExecuteCartesianPath(arm_group_, waypoints, "moveToTargetPose", eef_step))
        {
            ROS_WARN("moveToTargetPose: Primary Cartesian planning failed. Trying joint-space fallback...");
            arm_group_.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    ROS_INFO("moveToTargetPose: Fallback joint-space planning succeeded.");
                    return true;
                }
                else
                {
                    ROS_ERROR("moveToTargetPose: Fallback execution failed.");
                    return false;
                }
            }
            else
            {
                ROS_ERROR("moveToTargetPose: Fallback planning failed.");
                return false;
            }
        }

        ROS_INFO("Successfully moved to target pose");
        return true;
    }

    // Move to a pre-grasp pose above the target object
    bool moveToPreGraspPose(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &target_pose,
        double preferred_height_offset,
        double eef_step,
        int max_attempts)
    {
        ROS_INFO("Attempting to move to pre-grasp position with preferred height %.3f m...",
                 preferred_height_offset);

        // Store the current pose for recovery if needed
        geometry_msgs::PoseStamped start_pose = arm_group.getCurrentPose();

        // Create a copy of the target pose
        geometry_msgs::PoseStamped pre_grasp_pose = target_pose;

        // Strategy 1: Direct above with preferred height
        pre_grasp_pose.pose.position.z += preferred_height_offset;
        if (moveToTargetPose(arm_group, pre_grasp_pose, eef_step))
        {
            ROS_INFO("Succeeded with preferred height offset (%.3f m)", preferred_height_offset);
            return true;
        }
        else
        {
            ROS_WARN("Failed with preferred height offset (%.3f m), moving to next strategy", preferred_height_offset);
        }

        // Strategy 2: Try increasing heights
        double height_increments[] = {0.15, 0.18, 0.20, 0.25};
        for (double height : height_increments)
        {
            if (height <= preferred_height_offset)
                continue;

            pre_grasp_pose = target_pose;
            pre_grasp_pose.pose.position.z += height;

            ROS_INFO("Trying increased height offset (%.3f m)...", height);
            if (moveToTargetPose(arm_group, pre_grasp_pose, eef_step))
            {
                ROS_INFO("Succeeded with increased height offset (%.3f m)", height);
                return true;
            }
        }
        ROS_WARN("Failed with increased height offsets, moving to next strategy");

        // Strategy 3: Try with X/Y offsets at different heights
        double xy_offsets[] = {0.03, 0.05, 0.07};
        std::vector<std::pair<double, double>> directions = {
            {1.0, 0.0},  // X+
            {-1.0, 0.0}, // X-
            {0.0, 1.0},  // Y+
            {0.0, -1.0}, // Y-
            {0.7, 0.7},  // Diagonal
            {0.7, -0.7}, // Diagonal
            {-0.7, 0.7}, // Diagonal
            {-0.7, -0.7} // Diagonal
        };

        for (double height : height_increments)
        {
            for (double offset : xy_offsets)
            {
                for (const auto &dir : directions)
                {
                    pre_grasp_pose = target_pose;
                    pre_grasp_pose.pose.position.z += height;
                    pre_grasp_pose.pose.position.x += dir.first * offset;
                    pre_grasp_pose.pose.position.y += dir.second * offset;

                    ROS_INFO("Trying offset [%.3f, %.3f, %.3f]...",
                             dir.first * offset, dir.second * offset, height);

                    if (moveToTargetPose(arm_group, pre_grasp_pose, eef_step))
                    {
                        ROS_INFO("Succeeded with offset [%.3f, %.3f, %.3f]",
                                 dir.first * offset, dir.second * offset, height);
                        return true;
                    }
                }
            }
        }

        ROS_WARN("Failed with all offset strategies, moving to next strategy");

        // Strategy 4: Last resort - try joint space planning
        ROS_WARN("Cartesian planning failed, trying joint space planning...");

        pre_grasp_pose = target_pose;
        pre_grasp_pose.pose.position.z += preferred_height_offset;

        // Set the pose target and run simple planning
        arm_group.setPoseTarget(pre_grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

        if (arm_group.plan(motion_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            if (arm_group.execute(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Succeeded with joint space planning");
                return true;
            }
        }

        ROS_ERROR("All pre-grasp positioning strategies failed");
        return false;
    }

    // Lower the arm vertically to a target pose
    bool lowerTheArm(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &current_pose,
        const geometry_msgs::PoseStamped &target_pose,
        const std::string &action_name,
        double eef_step,
        double min_safe_height)
    {
        ROS_INFO("%s: Attempting to lower arm to target position...", action_name.c_str());

        // Check if target height is too low and adjust if needed
        geometry_msgs::PoseStamped adjusted_target_pose = target_pose;

        // Ensure minimum safe height
        if (adjusted_target_pose.pose.position.z < min_safe_height)
        {
            ROS_WARN("%s: Target height (%.3fm) is below minimum safe height. Adjusting to %.3fm",
                     action_name.c_str(), adjusted_target_pose.pose.position.z, min_safe_height);
            adjusted_target_pose.pose.position.z = min_safe_height;
        }

        // Calculate how far to lower from current position
        double descent_amount = current_pose.pose.position.z - adjusted_target_pose.pose.position.z;
        ROS_INFO("%s: Lowering from %.3f to %.3f (%.3fm descent)",
                 action_name.c_str(), current_pose.pose.position.z,
                 adjusted_target_pose.pose.position.z, descent_amount);

        // Attempt direct approach
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(adjusted_target_pose.pose);
        if (planAndExecuteCartesianPath(arm_group, waypoints, action_name + "_direct", eef_step))
        {
            return true;
        }

        ROS_WARN("%s: Failed to plan direct path, trying intermediate approach", action_name.c_str());
        geometry_msgs::PoseStamped intermediate_pose = current_pose;
        intermediate_pose.pose.position.z =
            current_pose.pose.position.z -
            (current_pose.pose.position.z - adjusted_target_pose.pose.position.z) * 0.7;

        ROS_INFO("%s: Trying intermediate point approach at z=%.3f",
                 action_name.c_str(), intermediate_pose.pose.position.z);

        std::vector<geometry_msgs::Pose> intermediate_waypoints;
        intermediate_waypoints.push_back(intermediate_pose.pose);
        if (!planAndExecuteCartesianPath(arm_group, intermediate_waypoints,
                                         action_name + "_intermediate", eef_step))
        {
            ROS_ERROR("%s: Failed to plan intermediate approach", action_name.c_str());
            return false;
        }

        std::vector<geometry_msgs::Pose> final_waypoints;
        final_waypoints.push_back(adjusted_target_pose.pose);
        if (!planAndExecuteCartesianPath(arm_group, final_waypoints,
                                         action_name + "_final", eef_step))
        {
            ROS_ERROR("%s: Failed to plan final approach from intermediate point", action_name.c_str());
            return false;
        }

        ROS_INFO("%s: Successfully lowered arm to target position", action_name.c_str());
        return true;
    }

    // Raise the arm vertically to a target pose
    bool raiseTheArm(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &current_pose,
        const geometry_msgs::PoseStamped &target_pose,
        const std::string &action_name,
        double eef_step,
        double max_safe_height)
    {
        ROS_INFO("%s: Attempting to raise arm to target position...", action_name.c_str());

        // Check if target height is too high and adjust if needed
        geometry_msgs::PoseStamped adjusted_target_pose = target_pose;

        // Ensure maximum safe height
        if (adjusted_target_pose.pose.position.z > max_safe_height)
        {
            ROS_WARN("%s: Target height (%.3fm) exceeds maximum safe height. Adjusting to %.3fm",
                     action_name.c_str(), adjusted_target_pose.pose.position.z, max_safe_height);
            adjusted_target_pose.pose.position.z = max_safe_height;
        }

        // Calculate how far to raise from current position
        double ascent_amount = adjusted_target_pose.pose.position.z - current_pose.pose.position.z;
        ROS_INFO("%s: Raising from %.3f to %.3f (%.3fm ascent)",
                 action_name.c_str(), current_pose.pose.position.z,
                 adjusted_target_pose.pose.position.z, ascent_amount);

        // Attempt direct approach
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(adjusted_target_pose.pose);
        if (planAndExecuteCartesianPath(arm_group, waypoints, action_name + "_direct", eef_step))
        {
            ROS_INFO("%s: Successfully raised arm to target position", action_name.c_str());
            return true;
        }

        ROS_WARN("%s: Failed to plan direct path, trying intermediate approach", action_name.c_str());

        // Create intermediate pose at 70% of the way up
        geometry_msgs::PoseStamped intermediate_pose = current_pose;
        intermediate_pose.pose.position.z =
            current_pose.pose.position.z +
            (adjusted_target_pose.pose.position.z - current_pose.pose.position.z) * 0.7;

        ROS_INFO("%s: Trying intermediate point approach at z=%.3f",
                 action_name.c_str(), intermediate_pose.pose.position.z);

        // First move to the intermediate position
        std::vector<geometry_msgs::Pose> intermediate_waypoints;
        intermediate_waypoints.push_back(intermediate_pose.pose);
        if (!planAndExecuteCartesianPath(arm_group, intermediate_waypoints,
                                         action_name + "_intermediate", eef_step))
        {
            ROS_ERROR("%s: Failed to plan intermediate approach", action_name.c_str());
            return false;
        }

        // Then complete the movement to the final position
        std::vector<geometry_msgs::Pose> final_waypoints;
        final_waypoints.push_back(adjusted_target_pose.pose);
        if (!planAndExecuteCartesianPath(arm_group, final_waypoints,
                                         action_name + "_final", eef_step))
        {
            ROS_ERROR("%s: Failed to plan final approach from intermediate point", action_name.c_str());
            return false;
        }

        ROS_INFO("%s: Successfully raised arm to target position", action_name.c_str());
        return true;
    }

    // Move the arm to a basket position with a hover height and approach distance
    bool transportToBasket(
        moveit::planning_interface::MoveGroupInterface &arm_group,
        const geometry_msgs::PoseStamped &basket_pose,
        double hover_height,
        double approach_distance)
    {
        ROS_INFO("Transporting object to basket position...");

        // Get current pose to preserve orientation
        geometry_msgs::PoseStamped current_pose = arm_group.getCurrentPose();

        // Create the hover pose by combining basket position with current orientation
        geometry_msgs::PoseStamped basket_hover_pose = basket_pose;
        basket_hover_pose.pose.position.z = hover_height;                   // Set desired height
        basket_hover_pose.pose.orientation = current_pose.pose.orientation; // Keep current orientation

        ROS_INFO("Moving to basket hover position at [%.3f, %.3f, %.3f]",
                 basket_hover_pose.pose.position.x,
                 basket_hover_pose.pose.position.y,
                 basket_hover_pose.pose.position.z);

        // Use moveToTargetPose which has joint-space fallback method
        if (!moveToTargetPose(arm_group, basket_hover_pose, 0.02))
        {
            // Even if execution fails, check if we're close enough to the target
            ROS_WARN("Execution failed, checking if current position is close enough to target...");

            // Get the current position after execution attempt
            geometry_msgs::PoseStamped actual_pose = arm_group.getCurrentPose();

            // Calculate distance from actual to target position
            double dx = actual_pose.pose.position.x - basket_hover_pose.pose.position.x;
            double dy = actual_pose.pose.position.y - basket_hover_pose.pose.position.y;
            double dz = actual_pose.pose.position.z - basket_hover_pose.pose.position.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            const double POSITION_TOLERANCE = 0.05; // 5cm tolerance

            if (distance <= POSITION_TOLERANCE)
            {
                ROS_INFO("Current position is within %.3fm of target (distance: %.3fm). Proceeding anyway.",
                         POSITION_TOLERANCE, distance);
            }
            else
            {
                ROS_ERROR("Failed to execute plan to basket hover position (distance: %.3fm)", distance);
                return false;
            }
        }

        ROS_INFO("Successfully reached basket hover position");
        return true;
    }

} // end namespace cw2_movement

#include <cw2_class.h>
#include <cw2_movement.h>
#include <cw2_perception.h>
#include <cw2_grasp.h>
/**
 * @file cw2_class.cpp
 * @brief Implementation of the cw2 class for the robot manipulation coursework
 * 
 * This file implements a ROS node that controls a Franka Emika Panda robot arm
 * to perform three main tasks:
 * 
 * 1. Task 1: Detect, grasp and relocate a specific object from a given position
 *    to a target location.
 * 
 * 2. Task 2: Identify reference shapes and classify an unknown "mystery shape"
 *    by comparing it to the reference shapes.
 * 
 * 3. Task 3: Scan the entire workspace, identify all objects (noughts, crosses,
 *    obstacles, and baskets), and pick up the most common shape type to place
 *    in the basket while avoiding obstacles.
 * 
 * The implementation uses point cloud processing for perception, MoveIt for
 * motion planning, and custom algorithms for shape detection and grasp planning.
 * It maintains an internal representation of all objects in the workspace and
 * visualizes them using RViz markers.
 */
cw2::cw2(ros::NodeHandle nh) : tfBuffer_(ros::Duration(15)), tfListener_(tfBuffer_)
{
  /* class constructor */
  nh_ = nh;
  base_frame_ = "panda_link0";

  // advertise solutions for coursework tasks
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

  // Publish PCL marker for visualization
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/task3/cluster_markers", 10);

  // Set planning parameters.
  arm_group_.setPlanningTime(25.0);
  arm_group_.setNumPlanningAttempts(1000);
  arm_group_.allowReplanning(true);

  // Increase tolerances to help overcome small deviations.
  arm_group_.setGoalJointTolerance(0.01);
  arm_group_.setGoalPositionTolerance(0.01);
  arm_group_.setGoalOrientationTolerance(0.01);

  // Initialize PCL components
  cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

  // Subscribe to the point cloud topic
  pointcloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);

  // Capture the default pose at startup.
  default_pose_ = arm_group_.getCurrentPose();

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response)
{
  ROS_INFO("Task 1 callback has been triggered");

  // Clear previous objects and planning scene
  all_objects_.clear();
  all_markers_.clear();
  next_marker_id_ = 0;
  cw2_perception::clearDebugImages();
  clearPlanningScene();

  ROS_INFO("Object position: [%.3f, %.3f, %.3f], Goal position: [%.3f, %.3f, %.3f], Shape type: %s",
           request.object_point.point.x, request.object_point.point.y, request.object_point.point.z,
           request.goal_point.point.x, request.goal_point.point.y, request.goal_point.point.z,
           request.shape_type.c_str());

  // Step 1: Move to observation pose over the object
  geometry_msgs::PoseStamped observe_pose;
  observe_pose.header.frame_id = "panda_link0";
  observe_pose.header.stamp = ros::Time::now();
  observe_pose.pose.position.x = request.object_point.point.x;
  observe_pose.pose.position.y = request.object_point.point.y;
  observe_pose.pose.position.z = 0.7; // 70cm above the table

  // Set downward-looking orientation
  tf2::Quaternion q;
  q.setRPY(0, M_PI, M_PI_4 + M_PI_2); // Roll=0, Pitch=π (looking down), Yaw=π/4+π/2
  q.normalize();
  observe_pose.pose.orientation = tf2::toMsg(q);

  if (!cw2_movement::moveToObservationPose(arm_group_, observe_pose,0.01))
  {
    ROS_ERROR("Failed to move to observation pose");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Wait for camera to stabilize and update point cloud
  ROS_INFO("Waiting for camera to stabilize...");
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Step 2: Get point cloud clusters and find the target object
  double min_height = 0.05, max_height = 0.15; // z-range for filtering clusters
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters =
      cw2_perception::getAllClusters(cloud_ptr_, tfBuffer_, min_height, max_height);

  // Find the cluster closest to the object_point
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cluster;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto &cluster : clusters)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    double dx = centroid[0] - request.object_point.point.x;
    double dy = centroid[1] - request.object_point.point.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance)
    {
      min_distance = distance;
      target_cluster = cluster;
    }
  }

  if (!target_cluster || target_cluster->empty() || min_distance > 0.15)
  {
    ROS_ERROR("No suitable object cluster found near target position");
    return true;
  }

  // Step 3: Detect the orientation of the object
  cv::Mat binaryImage = cw2_perception::clusterToBinaryImage(target_cluster, 0.003, true);
  double orientation = cw2_perception::getShapeOrientation(binaryImage, true);
  ROS_INFO("Detected object orientation: %.2f degrees", orientation);

  // Add object to our tracking system
  cw2::ObjectInfo obj;
  obj.cluster = target_cluster;
  obj.shapeType = request.shape_type;
  obj.orientation = orientation;

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*target_cluster, centroid);

  obj.pose.header.frame_id = "panda_link0";
  obj.pose.header.stamp = ros::Time::now();
  obj.pose.pose.position.x = centroid[0];
  obj.pose.pose.position.y = centroid[1];
  obj.pose.pose.position.z = centroid[2];
  obj.pose.pose.orientation.w = 1.0;

  obj.isAccessible = true;
  obj.uniqueId = "task1_object";
  all_objects_.push_back(obj);

  // Visualize the object cluster
  publishAllObjectMarkers();

  // Step 4: Calculate grasp pose based on shape type and orientation
  ROS_INFO("Planning grasp for %s", request.shape_type.c_str());

  // Get parameter values for grasping
  double pre_grasp_offset;
  if (!nh_.getParam("pre_grasp_offset", pre_grasp_offset))
  {
    pre_grasp_offset = 0.125; // Default if not set
  }

  double grasp_offset;
  if (!nh_.getParam("grasp_offset", grasp_offset))
  {
    grasp_offset = 0.15; // Default if not set
  }

  double lift_height;
  if (!nh_.getParam("lift_height", lift_height))
  {
    lift_height = 0.5; // Default if not set
  }

  double drop_height;
  if (!nh_.getParam("drop_height", drop_height))
  {
    drop_height = 0.6; // Default if not set
  }

  double lateral_offset;
  if (!nh_.getParam("lateral_offset", lateral_offset))
  {
    lateral_offset = 0.05; // Default if not set
  }

  // Compute grasp pose based on shape type and orientation
  geometry_msgs::PoseStamped grasp_pose = cw2_grasp::computeGraspPose(obj, lateral_offset);
  publishGraspPoseMarker(grasp_pose);

  // Step 5: Open gripper
  ROS_INFO("Opening gripper...");
  if (!cw2_grasp::prepareGripperOpen(hand_group_, gripper_open_))
  {
    ROS_ERROR("Failed to open gripper");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 6: Move to pre-grasp position (higher above the object)
  ROS_INFO("Moving to pre-grasp position...");
  if (!cw2_movement::moveToPreGraspPose(arm_group_, grasp_pose, pre_grasp_offset, 0.01, 5))
  {
    ROS_ERROR("Failed to move to pre-grasp position");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 7: Lower to grasp position
  ROS_INFO("Lowering to grasp position...");
  if (!cw2_movement::lowerTheArm(arm_group_, arm_group_.getCurrentPose(), grasp_pose, "lowerToGraspPose", 0.005, grasp_offset))
  {
    ROS_ERROR("Failed to lower to grasp position using fallback strategies");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 8: Close gripper to grasp object
  ROS_INFO("Closing gripper to grasp object...");
  if (!cw2_grasp::closeGripperForGrasp(hand_group_, gripper_closed_))
  {
    ROS_ERROR("Failed to close gripper");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 9: Raise the arm with the object
  ROS_INFO("Raising the arm with object...");
  geometry_msgs::PoseStamped raise_pose = grasp_pose;
  raise_pose.pose.position.z += lift_height;

  if (!cw2_movement::raiseTheArm(arm_group_, arm_group_.getCurrentPose(), raise_pose, "raiseArmWithObject", 0.01, 0.8))
  {
    ROS_ERROR("Failed to raise the arm with object");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 10: Move to goal position
  ROS_INFO("Moving to goal basket position...");
  geometry_msgs::PoseStamped basket_pose;
  basket_pose.header.frame_id = "panda_link0";
  basket_pose.pose.position = request.goal_point.point;
  basket_pose.pose.orientation.w = 1.0;

  if (!cw2_movement::transportToBasket(arm_group_, basket_pose, drop_height, 0.15))
  {
    ROS_ERROR("Failed to move to basket position");
    cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
    return true;
  }

  // Step 11: Lower the arm to a better drop height
  ROS_INFO("Lowering arm to optimal drop height...");
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  geometry_msgs::PoseStamped lower_pose = current_pose;
  lower_pose.pose.position.z = 0.4;

  if (!cw2_movement::lowerTheArm(arm_group_, current_pose, lower_pose, "lowerToDropHeight", 0.005, 0.15))
  {
    ROS_WARN("Failed to lower to optimal drop height, continuing with release anyway");
    // Continue with the task even if this specific intermediate step fails
  }
  else
  {
    ROS_INFO("Successfully lowered to optimal drop height");
    // Small pause to stabilize
    ros::Duration(0.2).sleep();
  }

  // Step 11: Release object
  ROS_INFO("Releasing object...");
  if (!cw2_grasp::releaseObject(hand_group_, gripper_open_))
  {
    ROS_ERROR("Failed to release object");
    return true;
  }

  // Step 12: Return to ready position
  ROS_INFO("Moving back to ready position...");
  cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 

  ROS_INFO("Task 1 completed successfully");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
                      cw2_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2 callback has been triggered");

  // First, clear all previously tracked objects
  all_objects_.clear();
  all_markers_.clear();
  next_marker_id_ = 0;
  cw2_perception::clearDebugImages();

  // NOTE TO EVALUATORS: Enable this line to move robot to 'ready' position
  // Make sure robot is in default pose
  // cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01);

  // Process reference shapes
  std::vector<std::string> ref_shapes;
  ROS_INFO("Processing %lu reference shapes", request.ref_object_points.size());

  // First detect the table height
  double table_height = cw2_perception::detectTableHeight(cloud_ptr_, tfBuffer_);
  ROS_INFO("Table height detected: %.3f meters", table_height - 0.02);

  // Define observation height
  const double observation_height = 0.7;

  // Set the z-range for filtering clusters
  double min_height = 0.05 + table_height - 0.02;
  double max_height = 1.5;

  for (size_t i = 0; i < request.ref_object_points.size(); i++)
  {
    const auto &point = request.ref_object_points[i];
    ROS_INFO("Reference shape %lu position: [%.3f, %.3f, %.3f]",
             i + 1, point.point.x, point.point.y, point.point.z);

    // Create observation pose above the reference point
    geometry_msgs::PoseStamped observe_pose;
    observe_pose.header.frame_id = "panda_link0";
    observe_pose.header.stamp = ros::Time::now();
    observe_pose.pose.position.x = point.point.x;
    observe_pose.pose.position.y = point.point.y;
    observe_pose.pose.position.z = observation_height;

    // Set downward-looking orientation
    tf2::Quaternion q;
    q.setRPY(0, M_PI, M_PI_4 + M_PI_2);
    q.normalize();
    observe_pose.pose.orientation = tf2::toMsg(q);

    // Move to observation pose
    ROS_INFO("Moving to observation pose for reference shape %lu", i + 1);
    if (!cw2_movement::moveToObservationPose(arm_group_, observe_pose, 0.01))
    {
      ROS_ERROR("Failed to move to observation pose for reference %lu", i + 1);
      continue;
    }

    // Wait for camera to stabilize and update point cloud
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    // Get all clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> nearby_clusters =
        cw2_perception::getAllClusters(cloud_ptr_, tfBuffer_, min_height, max_height);

    // Find the cluster closest to the specified point
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto &candidate_cluster : nearby_clusters)
    {
      // Calculate centroid of this cluster
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*candidate_cluster, centroid);

      // Calculate distance from centroid to the specified point
      double dx = centroid[0] - point.point.x;
      double dy = centroid[1] - point.point.y;
      double dz = centroid[2] - point.point.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      // If this cluster is closer, select it
      if (distance < min_distance)
      {
        min_distance = distance;
        cluster = candidate_cluster;
      }
    }

    // Only proceed if we found a cluster within a reasonable distance
    if (min_distance > 0.15)
    { // 15cm threshold
      ROS_ERROR("No cluster found close enough to reference %lu", i + 1);
      continue;
    }

    if (!cluster || cluster->points.empty())
    {
      ROS_ERROR("No valid cluster found for reference %lu", i + 1);
      continue;
    }

    // Convert to binary image and classify
    cv::Mat binaryImage = cw2_perception::clusterToBinaryImage(cluster, 0.003, true);
    bool is_nought = cw2_perception::isNought(binaryImage);
    std::string shape_type = is_nought ? "nought" : "cross";
    ref_shapes.push_back(shape_type);

    ROS_INFO("Reference shape %lu classified as: %s", i + 1, shape_type.c_str());

    // Add to our objects collection for visualization
    double orientation = cw2_perception::getShapeOrientation(binaryImage, true);
    cw2::addOrUpdateObject(cluster, shape_type, orientation);

    // Publish markers
    publishAllObjectMarkers();
    // publishDebugClusters(nearby_clusters);
  }

  // Move back to ready position for preventing failure
  cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 

  // Now process the mystery shape
  ROS_INFO("Processing mystery shape at position: [%.3f, %.3f, %.3f]",
           request.mystery_object_point.point.x,
           request.mystery_object_point.point.y,
           request.mystery_object_point.point.z);

  // Create observation pose above the mystery point
  geometry_msgs::PoseStamped observe_pose;
  observe_pose.header.frame_id = "panda_link0";
  observe_pose.header.stamp = ros::Time::now();
  observe_pose.pose.position.x = request.mystery_object_point.point.x;
  observe_pose.pose.position.y = request.mystery_object_point.point.y;
  observe_pose.pose.position.z = observation_height;

  // Set downward-looking orientation
  tf2::Quaternion q;
  q.setRPY(0, M_PI, M_PI_4 + M_PI_2); // Roll=0, Pitch=π (looking down), Yaw=0
  q.normalize();
  observe_pose.pose.orientation = tf2::toMsg(q);

  // Move to observation pose
  ROS_INFO("Moving to observation pose for mystery shape");
  if (!cw2_movement::moveToObservationPose(arm_group_, observe_pose, 0.01))
  {
    ROS_ERROR("Failed to move to observation pose for mystery shape");
    response.mystery_object_num = 0; // Error value
    return true;
  }

  // Wait for camera to stabilize and update point cloud
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Get all clusters
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> nearby_clusters =
      cw2_perception::getAllClusters(cloud_ptr_, tfBuffer_, min_height, max_height);

  // Find the cluster closest to the specified point
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto &candidate_cluster : nearby_clusters)
  {
    // Calculate centroid of this cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*candidate_cluster, centroid);

    // Calculate distance from centroid to the specified point
    double dx = centroid[0] - request.mystery_object_point.point.x;
    double dy = centroid[1] - request.mystery_object_point.point.y;
    double dz = centroid[2] - request.mystery_object_point.point.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // If this cluster is closer, select it
    if (distance < min_distance)
    {
      min_distance = distance;
      cluster = candidate_cluster;
    }
  }

  // Only proceed if we found a cluster within a reasonable distance
  if (min_distance > 0.15) // 15cm threshold
  {
    ROS_ERROR("No cluster found close enough to mystery shape");
    response.mystery_object_num = 0; // Error value
    return true;
  }

  if (!cluster || cluster->points.empty())
  {
    ROS_ERROR("No valid cluster found for mystery shape");
    response.mystery_object_num = 0; // Error value
    return true;
  }

  // Convert to binary image and classify
  cv::Mat binaryImage = cw2_perception::clusterToBinaryImage(cluster, 0.003, true);
  bool is_nought = cw2_perception::isNought(binaryImage);
  std::string mystery_shape_type = is_nought ? "nought" : "cross";

  ROS_INFO("Mystery shape classified as: %s", mystery_shape_type.c_str());

  // Add to our objects collection for visualization
  double orientation = cw2_perception::getShapeOrientation(binaryImage, true);
  cw2::addOrUpdateObject(cluster, mystery_shape_type, orientation);

  // publishDebugClusters(nearby_clusters);

  // ros::Duration(2.0).sleep();

  // Publish markers for all objects
  publishAllObjectMarkers();

  // Find which reference object the mystery object matches
  response.mystery_object_num = 0; // Default to error value
  for (size_t i = 0; i < ref_shapes.size(); i++)
  {
    if (ref_shapes[i] == mystery_shape_type)
    {
      response.mystery_object_num = i + 1; // 1-based indexing as requested
      ROS_INFO("Mystery shape matches reference %lu (%s)",
               i + 1, ref_shapes[i].c_str());
      break;
    }
  }

  // Return to ready position
  cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 

  ROS_INFO("Task 2 completed. Response: mystery_object_num = %ld", response.mystery_object_num);
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
                      cw2_world_spawner::Task3Service::Response &response)
{

  ROS_INFO("Task 3 callback has been triggered");

  // arm_group_.setPlanningTime(25.0);                // Allow up to 25 seconds for planning (instead of default 5)
  // arm_group_.setMaxVelocityScalingFactor(0.5);     // Slow down movements for more reliable execution
  // arm_group_.setMaxAccelerationScalingFactor(0.5); // Lower acceleration for smoother motion

  // Get the list of observation poses for scanning
  std::vector<geometry_msgs::PoseStamped> observation_poses = cw2_movement::getObservationPoses();
  ROS_INFO("Generated %lu observation poses", observation_poses.size());

  // Initialize global object tracking collections
  all_objects_.clear();
  all_markers_.clear();
  next_marker_id_ = 0;

  // Clear debug images
  cw2_perception::clearDebugImages();

  // Clear previous planning scene
  clearPlanningScene();

  // Visit each observation pose
  for (size_t i = 0; i < observation_poses.size(); i++)
  {
    ROS_INFO("Moving to observation pose %lu of %lu", i + 1, observation_poses.size());

    if (!cw2_movement::moveToObservationPose(arm_group_, observation_poses[i], 0.01))
    {
      ROS_ERROR("Failed to reach observation pose %lu, skipping...", i + 1);
      continue;
    }
    ROS_INFO("Successfully reached observation pose %lu", i + 1);

    // Wait for camera to stabilize and update point cloud
    ROS_INFO("Waiting 1 second for camera to stabilize...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    // Process the point cloud from this viewpoint
    ROS_INFO("Processing point cloud from observation pose %lu...", i + 1);

    // Define the z-range for filtering clusters
    double min_height = 0.05, max_height = 1.5;

    // Get all clusters from the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters =
        cw2_perception::getAllClusters(cloud_ptr_, tfBuffer_, min_height, max_height);

    // Process each cluster
    for (const auto &cluster : clusters)
    {
      // Calculate centroid in world frame
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);

      // Classify object
      std::string shapeType;
      double orientation;

      if (cw2_perception::isObstacle(cluster))
      {
        shapeType = "obstacle";
        orientation = 0.0;
      }
      else if (cw2_perception::isBasket(cluster))
      {
        shapeType = "basket";
        orientation = 0.0;
      }
      else
      {
        cv::Mat binaryImage = cw2_perception::clusterToBinaryImage(cluster, 0.003);
        shapeType = cw2_perception::isNought(binaryImage) ? "nought" : "cross";
        orientation = cw2_perception::getShapeOrientation(binaryImage, true);
      }

      // Add new object
      cw2::addOrUpdateObject(cluster, shapeType, orientation);
    }

    // Update visualization after each observation position
    cw2::publishAllObjectMarkers();

    ROS_INFO("Published all object markers...");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  // Count the different object types
  int64_t nought_count = 0;
  int64_t cross_count = 0;
  int64_t obstacle_count = 0;
  int64_t basket_count = 0;
  geometry_msgs::PoseStamped basket_position;

  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType == "nought")
    {
      nought_count++;
    }
    else if (obj.shapeType == "cross")
    {
      cross_count++;
    }
    else if (obj.shapeType == "obstacle")
    {
      obstacle_count++;
    }
    else if (obj.shapeType == "basket")
    {
      basket_count++;
      basket_position = obj.pose;
    }
  }

  // Log the counts
  ROS_INFO("Task 3 scan complete. Detected objects:");
  ROS_INFO("- Noughts: %ld", nought_count);
  ROS_INFO("- Crosses: %ld", cross_count);
  ROS_INFO("- Obstacles: %ld", obstacle_count);
  ROS_INFO("- Baskets: %ld", basket_count);

  // Add detailed orientation info for each object
  ROS_INFO("Object Orientations Details:");
  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType == "nought" || obj.shapeType == "cross")
    {
      ROS_INFO("%s (%s): Orientation = %.2f degrees",
               obj.uniqueId.c_str(),
               obj.shapeType.c_str(),
               obj.orientation);
    }
  }

  // Determine which shape is more common
  std::string more_common_shape;
  if (cross_count > nought_count)
  {
    more_common_shape = "cross";
    ROS_INFO("More common shape: Cross");
  }
  else if (nought_count > cross_count)
  {
    more_common_shape = "nought";
    ROS_INFO("More common shape: Nought");
  }
  else
  {
    more_common_shape = "equal";
    ROS_INFO("Equal number of noughts and crosses detected: %ld each", cross_count);
  }

  if (basket_count == 0)
  {
    ROS_WARN("No basket detected! Cannot complete pick and place task.");
  }
  else
  {
    ROS_INFO("Basket found at position: [%.3f, %.3f, %.3f]",
             basket_position.pose.position.x,
             basket_position.pose.position.y,
             basket_position.pose.position.z);
  }

  // Fill the response with the detected information
  response.total_num_shapes = nought_count + cross_count;
  if (more_common_shape == "cross")
  {
    response.num_most_common_shape = cross_count;
  }
  else if (more_common_shape == "nought")
  {
    response.num_most_common_shape = nought_count;
  }
  else
  {
    response.num_most_common_shape = cross_count;
  }

  ROS_INFO("Task 3 response: total_num_shapes=%ld, num_most_common_shape=%ld",
           response.total_num_shapes, response.num_most_common_shape);

  // Add all detected obstacles to the planning scene
  int obstacles_added = 0;
  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType == "obstacle")
    {
      addObstacleToPlanningScene(obj);
      obstacles_added++;
    }
  }

  // Wait for the planning scene to update
  ros::Duration(0.5).sleep();
  ROS_INFO("Added %d obstacles to planning scene - continuing with pick and place", obstacles_added);

  // Return to "ready" position after completing the task
  cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 

  // After moving back to the ready position and adding obstacles to the planning scene:
  // Only proceed if at least one basket was found
  if (basket_count > 0)
  {
    // Get multiple candidate objects of the most common shape
    std::vector<std::pair<const cw2::ObjectInfo *, double>> candidate_objects;

    if (more_common_shape == "cross")
    {
      candidate_objects = selectBestObjects("cross", 4); // Get up to 4 best crosses
    }
    else if (more_common_shape == "nought")
    {
      candidate_objects = selectBestObjects("nought", 4); // Get up to 4 best noughts
    }
    else
    {
      // Equal number - try crosses first, then noughts
      candidate_objects = selectBestObjects("cross", 3);
      if (candidate_objects.empty())
      {
        candidate_objects = selectBestObjects("nought", 3);
      }
    }

    ROS_INFO("Found %lu candidate objects to try for picking", candidate_objects.size());

    // Variables to track pick and place success
    bool pick_success = false;
    int attempts = 0;

    // Try each candidate in order until success or we run out of options
    for (const auto &[object_to_pick, score] : candidate_objects)
    {

      if (!object_to_pick)
        continue;
      attempts++;

      ROS_INFO("Attempt %d/%lu: Planning to pick up %s object %s (score: %.3f)",
               attempts, candidate_objects.size(),
               object_to_pick->shapeType.c_str(),
               object_to_pick->uniqueId.c_str(),
               score);

      // Convert object centroid to a point stamped for motion planning
      geometry_msgs::PointStamped object_point;
      object_point.header = object_to_pick->pose.header;
      object_point.point = object_to_pick->pose.pose.position;

      // Create point stamped for basket location
      geometry_msgs::PointStamped basket_point;
      basket_point.header = basket_position.header;
      basket_point.point = basket_position.pose.position;

      double lateral_offset;
      if (!nh_.getParam("lateral_offset", lateral_offset))
      {
        lateral_offset = 0.05; // 1cm default offset to the right
        ROS_INFO("Using default lateral_offset: %.3f m", lateral_offset);
      }

      // Compute grasp pose using orientation info
      geometry_msgs::PoseStamped grasp_pose = cw2_grasp::computeGraspPose(*object_to_pick, lateral_offset);

      // Visualize the grasp pose to verify it
      publishGraspPoseMarker(grasp_pose);

      // Wait to show the marker
      ROS_INFO("Showing grasp pose marker - waiting 0.5 seconds before proceeding");
      ros::Duration(0.5).sleep();

      // Track if all steps in the sequence succeed
      bool sequence_success = true;

      // 1. Open gripper before approaching
      ROS_INFO("Opening gripper...");
      if (!cw2_grasp::prepareGripperOpen(hand_group_, gripper_open_))
      {
        ROS_ERROR("Failed to open gripper");
        sequence_success = false;
        continue; // Try next object
      }

      // 2. Move to pre-grasp position
      double pre_grasp_offset;
      if (!nh_.getParam("pre_grasp_offset", pre_grasp_offset))
      {
        pre_grasp_offset = 0.15; // Default value
      }

      ROS_INFO("Moving to pre-grasp position...");
      if (!cw2_movement::moveToPreGraspPose(arm_group_, grasp_pose, pre_grasp_offset, 0.01, 5))
      {
        ROS_ERROR("Failed to move to pre-grasp position");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // 3. Lower to grasp position
      double grasp_offset;
      if (!nh_.getParam("grasp_offset", grasp_offset))
      {
        grasp_offset = 0.15; // Default value
      }

      ROS_INFO("Lowering to grasp position...");
      if (!cw2_movement::lowerTheArm(arm_group_, arm_group_.getCurrentPose(), grasp_pose, "lowerToGraspPose", 0.005, grasp_offset))
      {
        ROS_ERROR("Failed to lower to grasp position");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // 4. Close gripper to grasp object
      ROS_INFO("Closing gripper to grasp object...");
      if (!cw2_grasp::closeGripperForGrasp(hand_group_, gripper_closed_))
      {
        ROS_ERROR("Failed to close gripper");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // 5. Lift object
      double lift_height;
      if (!nh_.getParam("lift_height", lift_height))
      {
        lift_height = 0.5; // Default lift height
      }

      ROS_INFO("Raising the arm with object...");
      geometry_msgs::PoseStamped raise_pose = grasp_pose;
      raise_pose.pose.position.z += lift_height;

      if (!cw2_movement::raiseTheArm(arm_group_, arm_group_.getCurrentPose(), raise_pose, "raiseArmWithObject", 0.01, 0.8))
      {
        ROS_ERROR("Failed to raise the arm with object");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // 6. Move to goal basket position
      double drop_height;
      if (!nh_.getParam("drop_height", drop_height))
      {
        drop_height = 0.6; // Default drop height
      }

      ROS_INFO("Moving to goal basket position...");
      if (!cw2_movement::transportToBasket(arm_group_, basket_position, drop_height, 0.15))
      {
        ROS_ERROR("Failed to move to basket position");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // 7. Lower the arm to a better drop height
      ROS_INFO("Lowering arm to optimal drop height...");
      geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
      geometry_msgs::PoseStamped lower_pose = current_pose;
      lower_pose.pose.position.z = 0.4;

      if (!cw2_movement::lowerTheArm(arm_group_, current_pose, lower_pose, "lowerToDropHeight", 0.005, 0.15))
      {
        ROS_WARN("Failed to lower to optimal drop height, continuing with release anyway");
        // Continue with the task even if this specific intermediate step fails
      }

      // 8. Release object
      ROS_INFO("Releasing object...");
      if (!cw2_grasp::releaseObject(hand_group_, gripper_open_))
      {
        ROS_ERROR("Failed to release object");
        sequence_success = false;
        cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 
        continue; // Try next object
      }

      // If we got here with all steps successful, we're done!
      if (sequence_success)
      {
        ROS_INFO("Successfully completed pick and place with object %s on attempt %d",
                 object_to_pick->uniqueId.c_str(), attempts);
        pick_success = true;
        break;
      }
    }

    if (!pick_success)
    {
      ROS_ERROR("Failed to pick and place any of the %lu candidate objects after %d attempts",
                candidate_objects.size(), attempts);
    }
  }
  else
  {
    ROS_ERROR("Cannot proceed with pick and place - no basket detected");
  }

  // Move back to default ready position
  ROS_INFO("Moving back to ready position...");
  cw2_movement::moveToReadyPosition(arm_group_, 30.0, 0.6, 0.6, 0.01); 

  ROS_INFO("Task 3 response: total_num_shapes=%ld, num_most_common_shape=%ld",
           response.total_num_shapes, response.num_most_common_shape);

  ROS_INFO("Task 3 completed successfully");

  return true;
}

////////////////////////////////////////////////////////////////////////////
////////////////////// HELPER FUNCTIONS/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Add point cloud callback
void cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl::fromROSMsg(*cloud_msg, *cloud_ptr_);
}

// Check if the new object is a new one or an existing one
bool cw2::isNewObject(
    const Eigen::Vector4f &new_centroid,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &new_cluster,
    int &existing_index,
    double distance_threshold)
{
  existing_index = -1; // Default: not found

  for (size_t i = 0; i < all_objects_.size(); ++i)
  {
    // Get existing object's centroid
    Eigen::Vector4f existing_centroid;
    pcl::compute3DCentroid(*(all_objects_[i].cluster), existing_centroid);

    // Calculate Euclidean distance between centroids
    double dx = new_centroid[0] - existing_centroid[0];
    double dy = new_centroid[1] - existing_centroid[1];
    double dz = new_centroid[2] - existing_centroid[2];
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // If centroids are close, consider it the same object
    if (distance < distance_threshold)
    {
      existing_index = i; // Save index of existing object
      return false;       // Not a new object
    }
  }
  return true; // New object
}

// Add or update object in the collection
void cw2::addOrUpdateObject(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
    const std::string &shapeType,
    double orientation)
{

  // Calculate centroid and create pose
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  // Check if this is a new object or existing one
  int existing_index;
  if (!cw2::isNewObject(centroid, cluster, existing_index, 0.15))
  {
    // Existing object found - update it if the new cluster has more points
    auto &existing_obj = all_objects_[existing_index];

    if (cluster->size() > existing_obj.cluster->size() * 1.2)
    { // 20% more points
      ROS_INFO("addOrUpdateObject: Updating object %s with better data (%lu points vs %lu points)",
               existing_obj.uniqueId.c_str(), cluster->size(), existing_obj.cluster->size());

      // Update the cluster
      existing_obj.cluster = cluster;

      // Update centroid
      existing_obj.pose.pose.position.x = centroid[0];
      existing_obj.pose.pose.position.y = centroid[1];
      existing_obj.pose.pose.position.z = centroid[2];

      // Update binary image
      existing_obj.binaryImage = cw2_perception::clusterToBinaryImage(cluster, 0.003);

      // Update orientation with new value
      existing_obj.orientation = orientation;

      // Update shape type if this is a more complete view
      // Only update if the object is not an obstacle or basket (those classifications are reliable)
      if (existing_obj.shapeType != "obstacle" && existing_obj.shapeType != "basket")
      {
        // Reclassify the shape with the better data
        bool is_nought = cw2_perception::isNought(existing_obj.binaryImage);

        // Update the shape type
        std::string new_shape = is_nought ? "nought" : "cross";

        // Log if changing classification
        if (existing_obj.shapeType != new_shape)
        {
          ROS_INFO("Updating object %s classification from %s to %s based on better data",
                   existing_obj.uniqueId.c_str(), existing_obj.shapeType.c_str(), new_shape.c_str());
          existing_obj.shapeType = new_shape;
        }
      }

      // Increment view count
      existing_obj.viewCount++;
    }
    return;
  }

  // Create new object info
  cw2::ObjectInfo obj;

  // Set point cloud
  obj.cluster = cluster;

  obj.pose.header.frame_id = cluster->header.frame_id;
  obj.pose.header.stamp = ros::Time::now();
  obj.pose.pose.position.x = centroid[0];
  obj.pose.pose.position.y = centroid[1];
  obj.pose.pose.position.z = centroid[2];
  obj.pose.pose.orientation.w = 1.0; // Identity quaternion

  // Create binary image projection
  obj.binaryImage = cw2_perception::clusterToBinaryImage(cluster, 0.003);

  // Store the orientation
  obj.orientation = orientation;

  // Set shape type
  obj.shapeType = shapeType;

  // Set other properties
  obj.isAccessible = true; // Will be checked properly later
  obj.viewCount = 1;
  obj.uniqueId = "obj_" + std::to_string(all_objects_.size() + 1);

  // Add to collection
  all_objects_.push_back(obj);

  ROS_INFO("addOrUpdateObject: Added new object %s of type %s at position [%.3f, %.3f, %.3f], orientation: %.1f deg",
           obj.uniqueId.c_str(), obj.shapeType.c_str(),
           centroid[0], centroid[1], centroid[2], orientation);
}

// Publish all object markers for visualization
void cw2::publishAllObjectMarkers()
{
  visualization_msgs::MarkerArray marker_array;

  // Delete all previous markers first
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  delete_marker.ns = "objects";
  marker_array.markers.push_back(delete_marker);
  marker_pub_.publish(marker_array);

  // Create new marker array
  marker_array.markers.clear();

  // Add markers for each object
  for (size_t i = 0; i < all_objects_.size(); ++i)
  {
    const auto &obj = all_objects_[i];

    // Create cube marker for object volume
    visualization_msgs::Marker cube_marker;
    cube_marker.header.frame_id = obj.pose.header.frame_id;
    cube_marker.header.stamp = ros::Time::now();
    cube_marker.ns = "objects";
    cube_marker.id = i * 2; // Even IDs for cubes
    cube_marker.type = visualization_msgs::Marker::CUBE;
    cube_marker.action = visualization_msgs::Marker::ADD;

    // Set position and size
    cube_marker.pose = obj.pose.pose;

    // Calculate dimensions
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*(obj.cluster), min_pt, max_pt);

    cube_marker.scale.x = max_pt[0] - min_pt[0] + 0.01;
    cube_marker.scale.y = max_pt[1] - min_pt[1] + 0.01;
    cube_marker.scale.z = max_pt[2] - min_pt[2] + 0.01;

    // Set color based on shape type
    if (obj.shapeType == "basket")
    {
      cube_marker.color.r = 0.6;
      cube_marker.color.g = 0.3;
      cube_marker.color.b = 0.1; // Brown for basket
    }
    else if (obj.shapeType == "nought")
    {
      cube_marker.color.r = 0.0;
      cube_marker.color.g = 1.0;
      cube_marker.color.b = 0.0; // Green
    }
    else if (obj.shapeType == "cross")
    {
      cube_marker.color.r = 0.0;
      cube_marker.color.g = 0.0;
      cube_marker.color.b = 1.0; // Blue
    }
    else
    {
      cube_marker.color.r = 1.0;
      cube_marker.color.g = 0.0;
      cube_marker.color.b = 0.0; // Red for obstacles
    }
    cube_marker.color.a = 0.3; // Semi-transparent

    // Add sphere marker for center point
    visualization_msgs::Marker sphere_marker;
    sphere_marker.header = cube_marker.header;
    sphere_marker.ns = "centers";
    sphere_marker.id = i * 2 + 1; // Odd IDs for spheres
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::Marker::ADD;
    sphere_marker.pose = obj.pose.pose;
    sphere_marker.scale.x = 0.02; // 2cm diameter sphere
    sphere_marker.scale.y = 0.02;
    sphere_marker.scale.z = 0.02;
    sphere_marker.color = cube_marker.color;
    sphere_marker.color.a = 1.0; // Fully opaque

    // Add text marker for ID
    visualization_msgs::Marker text_marker;
    text_marker.header = cube_marker.header;
    text_marker.ns = "labels";
    text_marker.id = i * 2 + 2; // Even + 2 for text
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose = obj.pose.pose;
    text_marker.pose.position.z += 0.05; // Slightly above center
    text_marker.text = obj.uniqueId + " (" + obj.shapeType + ")";
    text_marker.scale.z = 0.05; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    // Add orientation arrow marker (only for crosses and noughts)
    if (obj.shapeType == "nought" || obj.shapeType == "cross")
    {
      visualization_msgs::Marker arrow_marker;
      arrow_marker.header = cube_marker.header;
      arrow_marker.ns = "orientations";
      arrow_marker.id = i * 2 + 3; // Unique ID for arrows
      arrow_marker.type = visualization_msgs::Marker::ARROW;
      arrow_marker.action = visualization_msgs::Marker::ADD;

      // Set starting position at object center
      arrow_marker.pose = obj.pose.pose;

      // Convert orientation in degrees to radians
      double angle_rad = obj.orientation * M_PI / 180.0;

      // Calculate quaternion for arrow orientation
      tf2::Quaternion q;
      q.setRPY(0, 0, angle_rad);
      q.normalize();
      arrow_marker.pose.orientation = tf2::toMsg(q);

      // Arrow dimensions
      arrow_marker.scale.x = 0.15; // Arrow length
      arrow_marker.scale.y = 0.02; // Arrow width
      arrow_marker.scale.z = 0.02; // Arrow height

      // Arrow color (yellow)
      arrow_marker.color.r = 1.0;
      arrow_marker.color.g = 1.0;
      arrow_marker.color.b = 0.0;
      arrow_marker.color.a = 1.0;

      // Add to marker array
      marker_array.markers.push_back(arrow_marker);
    }

    // Add markers to array
    marker_array.markers.push_back(cube_marker);
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(text_marker);
  }

  // Publish all markers
  marker_pub_.publish(marker_array);
  ROS_INFO("Published visualization markers for %lu objects", all_objects_.size());
}

void cw2::addObstacleToPlanningScene(
    const ObjectInfo &obj)
{
  // Only add obstacles to planning scene
  if (obj.shapeType != "obstacle")
  {
    return;
  }

  // Create a collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = obj.pose.header.frame_id;
  collision_object.id = obj.uniqueId;

  // Get the min/max bounds of the obstacle
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*(obj.cluster), min_pt, max_pt);

  // Create a box primitive slightly larger than the actual obstacle
  const double SAFETY_MARGIN = 0.01; // 1cm safety margin
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = max_pt[0] - min_pt[0] + SAFETY_MARGIN * 2;
  primitive.dimensions[1] = max_pt[1] - min_pt[1] + SAFETY_MARGIN * 2;
  primitive.dimensions[2] = max_pt[2] - min_pt[2] + SAFETY_MARGIN * 2;

  // Define the box pose (center of the box)
  geometry_msgs::Pose box_pose;
  box_pose.position.x = (min_pt[0] + max_pt[0]) / 2.0;
  box_pose.position.y = (min_pt[1] + max_pt[1]) / 2.0;
  box_pose.position.z = (min_pt[2] + max_pt[2]) / 2.0;
  box_pose.orientation.w = 1.0; // Identity quaternion

  // Add the primitive and its pose to the collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add the collision object to the planning scene
  moveit_msgs::CollisionObject co = collision_object;
  planning_scene_interface_.applyCollisionObject(co);

  ROS_INFO("addObstacleToPlanningScene: Added obstacle %s to planning scene at [%.3f, %.3f, %.3f]",
           obj.uniqueId.c_str(), box_pose.position.x, box_pose.position.y, box_pose.position.z);
}

// Clear the planning scene of all obstacles
void cw2::clearPlanningScene()
{
  std::vector<std::string> object_ids;

  // Get all object IDs from your collection
  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType == "obstacle")
    {
      object_ids.push_back(obj.uniqueId);
    }
  }

  // Remove objects if there are any
  if (!object_ids.empty())
  {
    planning_scene_interface_.removeCollisionObjects(object_ids);
    ROS_INFO("clearPlanningScene: Cleared planning scene of %zu obstacles", object_ids.size());
  }
  else
  {
    ROS_INFO("clearPlanningScene: No obstacles to clear from planning scene");
  }
}

// Select the best object to pick based on multiple criteria
const cw2::ObjectInfo *cw2::selectBestObject(
    const std::string &shape_type)
{
  const cw2::ObjectInfo *best_object = nullptr;
  double best_score = -1.0;

  // Only consider objects of the specified type
  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType != shape_type)
    {
      continue;
    }

    // Skip objects that are not accessible
    if (!obj.isAccessible)
    {
      continue;
    }

    // Calculate score based on multiple factors
    double score = 0.0;

    // Factor 1: Distance from robot base (optimal is around 0.4m)
    double distance = std::sqrt(
        std::pow(obj.pose.pose.position.x, 2) +
        std::pow(obj.pose.pose.position.y, 2));

    double optimal_distance = 0.4; // The sweet spot for manipulation
    double distance_score = std::exp(-5.0 * std::pow(distance - optimal_distance, 2));

    // Factor 2: General clearance (more clearance is better)
    double clearance_score = calculateObjectClearance(obj);
    // double obstacle_score = min_obstacle_distance;

    // Factor 3: Orientation alignment (better aligned with the gripper is better)
    double orientation_score = calculateOrientationScore(obj);

    // Factor 4: Object size (larger objects are easier to grasp)
    double size_score = calculateSizeScore(obj);

    // Weight factors and calculate final score
    double final_score = 0.25 * distance_score +
                         0.30 * clearance_score +
                         0.20 * orientation_score +
                         0.25 * size_score;

    ROS_INFO("Object %s score: %.3f (dist=%.3f, clear=%.3f, orient=%.3f, size=%.3f)",
             obj.uniqueId.c_str(), final_score,
             distance_score, clearance_score, orientation_score, size_score);

    // Update best object if this one has a higher score
    if (final_score > best_score)
    {
      best_score = final_score;
      best_object = &obj;
    }
  }

  if (best_object)
  {
    ROS_INFO("Selected %s as best object with score %.3f at position [%.3f, %.3f, %.3f]",
             best_object->uniqueId.c_str(), best_score,
             best_object->pose.pose.position.x,
             best_object->pose.pose.position.y,
             best_object->pose.pose.position.z);
  }
  else
  {
    ROS_WARN("No accessible %s found for picking", shape_type.c_str());
  }

  return best_object;
}

// Returns a vector of objects sorted by score (best first)
std::vector<std::pair<const cw2::ObjectInfo *, double>> cw2::selectBestObjects(
    const std::string &shape_type,
    int max_candidates)
{
  std::vector<std::pair<const cw2::ObjectInfo *, double>> scored_objects;

  // Only consider objects of the specified type
  for (const auto &obj : all_objects_)
  {
    if (obj.shapeType != shape_type || !obj.isAccessible)
      continue;

    // Calculate score based on multiple factors

    // Factor 1: Distance from robot base (optimal is around 0.4m)
    double distance = std::sqrt(
        std::pow(obj.pose.pose.position.x, 2) +
        std::pow(obj.pose.pose.position.y, 2));

    double optimal_distance = 0.4; // The sweet spot for manipulation
    double distance_score = std::exp(-5.0 * std::pow(distance - optimal_distance, 2));

    // Factor 2: General clearance (more clearance is better)
    double clearance_score = calculateObjectClearance(obj);

    // Factor 3: Orientation alignment
    double orientation_score = calculateOrientationScore(obj);

    // Factor 4: Object size (larger objects are easier to grasp)
    double size_score = calculateSizeScore(obj);

    // Weight factors and calculate final score
    double final_score = 0.25 * distance_score +
                         0.30 * clearance_score +
                         0.20 * orientation_score +
                         0.25 * size_score;

    ROS_INFO("Object %s score: %.3f (dist=%.3f, clear=%.3f, orient=%.3f, size=%.3f)",
             obj.uniqueId.c_str(), final_score,
             distance_score, clearance_score, orientation_score, size_score);

    // Add this object to our scored list
    scored_objects.push_back({&obj, final_score});
  }

  // Sort objects by score (highest first)
  std::sort(scored_objects.begin(), scored_objects.end(),
            [](const auto &a, const auto &b)
            {
              return a.second > b.second;
            });

  // Limit the number of candidates if requested
  if (max_candidates > 0 && scored_objects.size() > static_cast<size_t>(max_candidates))
  {
    scored_objects.resize(max_candidates);
  }

  // Report the candidates found
  if (!scored_objects.empty())
  {
    ROS_INFO("Found %lu ranked %s candidates. Best score: %.3f for object %s",
             scored_objects.size(), shape_type.c_str(),
             scored_objects[0].second, scored_objects[0].first->uniqueId.c_str());
  }
  else
  {
    ROS_WARN("No accessible %s found for picking", shape_type.c_str());
  }

  return scored_objects;
}

// Calculate minimum distance from an object to any other object
double cw2::calculateObjectClearance(
    const cw2::ObjectInfo &obj)
{
  double min_distance = 1.0; // Initialize with a reasonable maximum (1 meter)

  for (const auto &other : all_objects_)
  {
    // Skip if comparing to self
    if (&other == &obj)
    {
      continue;
    }

    // Calculate Euclidean distance between centroids
    double dx = obj.pose.pose.position.x - other.pose.pose.position.x;
    double dy = obj.pose.pose.position.y - other.pose.pose.position.y;
    double dz = obj.pose.pose.position.z - other.pose.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Get object dimensions
    Eigen::Vector4f obj_min, obj_max, other_min, other_max;
    pcl::getMinMax3D(*(obj.cluster), obj_min, obj_max);
    pcl::getMinMax3D(*(other.cluster), other_min, other_max);

    // Approximate half-sizes
    double obj_size = std::max({(obj_max[0] - obj_min[0]) / 2,
                                (obj_max[1] - obj_min[1]) / 2,
                                (obj_max[2] - obj_min[2]) / 2});

    double other_size = std::max({(other_max[0] - other_min[0]) / 2,
                                  (other_max[1] - other_min[1]) / 2,
                                  (other_max[2] - other_min[2]) / 2});

    // Adjust distance by subtracting approximate object sizes
    double adjusted_distance = distance - obj_size - other_size;

    // Update minimum distance if this is smaller
    min_distance = std::min(min_distance, adjusted_distance);
  }

  // Normalize to [0,1] range (saturate at 0.2m)
  return std::min(1.0, min_distance / 0.2);
}

// Calculate how well aligned the object is with the gripper
double cw2::calculateOrientationScore(
    const cw2::ObjectInfo &obj)
{
  double orientation_rad = obj.orientation * M_PI / 180.0;
  double score = 0.0;

  // Normalize orientation to [0, π] range
  // This accounts for the fact that orientation is in the range [-180°, 180°]
  orientation_rad = std::fmod(std::abs(orientation_rad), M_PI);

  if (obj.shapeType == "cross")
  {
    // For crosses, prefer orientations close to 0, 45, 90, or 135 degrees
    // (these make grasping easier by aligning with a cardinal or diagonal direction)
    double mod_angle = std::fmod(orientation_rad, M_PI / 2);  // Fold to [0,π/2]
    double dist1 = std::min(mod_angle, M_PI / 2 - mod_angle); // Distance to 0 or 90
    double dist2 = std::abs(mod_angle - M_PI / 4);            // Distance to 45
    double min_dist = std::min(dist1, dist2);

    // Convert distance to score (smaller distance = higher score)
    score = 1.0 - min_dist / (M_PI / 4); // Normalize to [0,1]
  }
  else if (obj.shapeType == "nought")
  {
    // For noughts, prefer orientations close to 0, 90, 180, 270 degrees
    // (these make grasping easier by aligning with the flat edges)
    double mod_angle = std::fmod(orientation_rad, M_PI / 2);             // Fold to [0,π/2]
    double dist_to_cardinal = std::min(mod_angle, M_PI / 2 - mod_angle); // Distance to 0 or 90

    // Convert distance to score (smaller distance = higher score)
    score = 1.0 - dist_to_cardinal / (M_PI / 4); // Normalize to [0,1]
  }

  return score;
}

// Calculate a score based on object size (larger is better for grasping)
double cw2::calculateSizeScore(
    const cw2::ObjectInfo &obj)
{
  // Count points in the cluster
  size_t point_count = obj.cluster->size();

  // Get physical dimensions of the cluster
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*(obj.cluster), min_pt, max_pt);
  double width = max_pt[0] - min_pt[0];
  double height = max_pt[1] - min_pt[1];
  double depth = max_pt[2] - min_pt[2];

  // Debug print with detailed cluster information
  ROS_INFO("Object %s size info: points=%zu, dimensions=[%.3f, %.3f, %.3f]m, volume=%.5fm³",
           obj.uniqueId.c_str(), point_count,
           width, height, depth,
           width * height * depth);

  // Normalize to [0,1] range
  // Typical clusters have between 100 and 3000 points
  double min_points = 100.0;
  double max_points = 3000.0;

  // Linear normalization
  double normalized_size = (static_cast<double>(point_count) - min_points) /
                           (max_points - min_points);

  // Clamp to [0,1]
  return std::max(0.0, std::min(1.0, normalized_size));
}

// Visualize grasp pose with arrow marker
void cw2::publishGraspPoseMarker(
    const geometry_msgs::PoseStamped &grasp_pose)
{
  visualization_msgs::MarkerArray marker_array;

  // Create arrow marker for grasp pose
  visualization_msgs::Marker arrow_marker;
  arrow_marker.header = grasp_pose.header;
  arrow_marker.ns = "grasp_pose";
  arrow_marker.id = 0;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.pose = grasp_pose.pose;
  arrow_marker.pose.position.z += 0.1; // Position arrow above the grasp pose

  // Arrow dimensions - slightly larger than object arrows for visibility
  arrow_marker.scale.x = 0.20; // Arrow length
  arrow_marker.scale.y = 0.02; // Arrow width
  arrow_marker.scale.z = 0.02; // Arrow height

  // Purple color for grasp pose
  arrow_marker.color.r = 0.8;
  arrow_marker.color.g = 0.2;
  arrow_marker.color.b = 0.8;
  arrow_marker.color.a = 1.0;

  // Add text marker for label
  visualization_msgs::Marker text_marker;
  text_marker.header = grasp_pose.header;
  text_marker.ns = "grasp_pose";
  text_marker.id = 1;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose = grasp_pose.pose;
  text_marker.pose.position.z += 0.1; // Position text above the arrow
  text_marker.text = "Grasp Pose";
  text_marker.scale.z = 0.05; // Text height
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  // Add markers to array
  marker_array.markers.push_back(arrow_marker);
  marker_array.markers.push_back(text_marker);

  // Publish markers
  marker_pub_.publish(marker_array);
  ROS_INFO("Published grasp pose visualization marker");
}

void cw2::publishDebugClusters(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clusters)
{
  visualization_msgs::MarkerArray marker_array;

  // Delete all previous markers first
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  delete_marker.ns = "debug_clusters";
  marker_array.markers.push_back(delete_marker);
  marker_pub_.publish(marker_array);

  marker_array.markers.clear();

  // Create markers for each cluster with different colors
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    const auto &cluster = clusters[i];

    // Skip empty clusters
    if (cluster->empty())
      continue;

    // Calculate centroid and bounds
    Eigen::Vector4f centroid, min_pt, max_pt;
    pcl::compute3DCentroid(*cluster, centroid);
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    // Create a unique color for this cluster based on index
    std_msgs::ColorRGBA color;
    color.a = 0.6; // Semi-transparent

    // // Check if this is potentially a green table
    // if (cw2_perception::isGreenTable(cluster))
    // {
    //   // Use bright green for table parts
    //   color.r = 0.0;
    //   color.g = 1.0;
    //   color.b = 0.0;
    // }
    // else
    // {
    //   // For other objects, use HSV color cycle (converting to RGB)
    //   float hue = (i * 0.618034f); // Golden ratio for better distribution
    //   hue = hue - floorf(hue);     // Keep in [0,1]

    //   // Simple HSV to RGB conversion for visualization variety
    //   float h = hue * 6.0f;
    //   float c = 1.0f;
    //   float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));

    //   if (h < 1.0f)
    //   {
    //     color.r = c;
    //     color.g = x;
    //     color.b = 0.0f;
    //   }
    //   else if (h < 2.0f)
    //   {
    //     color.r = x;
    //     color.g = c;
    //     color.b = 0.0f;
    //   }
    //   else if (h < 3.0f)
    //   {
    //     color.r = 0.0f;
    //     color.g = c;
    //     color.b = x;
    //   }
    //   else if (h < 4.0f)
    //   {
    //     color.r = 0.0f;
    //     color.g = x;
    //     color.b = c;
    //   }
    //   else if (h < 5.0f)
    //   {
    //     color.r = x;
    //     color.g = 0.0f;
    //     color.b = c;
    //   }
    //   else
    //   {
    //     color.r = c;
    //     color.g = 0.0f;
    //     color.b = x;
    //   }
    // }

    // For other objects, use HSV color cycle (converting to RGB)
    float hue = (i * 0.618034f); // Golden ratio for better distribution
    hue = hue - floorf(hue);     // Keep in [0,1]

    // Simple HSV to RGB conversion for visualization variety
    float h = hue * 6.0f;
    float c = 1.0f;
    float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));

    if (h < 1.0f)
    {
      color.r = c;
      color.g = x;
      color.b = 0.0f;
    }
    else if (h < 2.0f)
    {
      color.r = x;
      color.g = c;
      color.b = 0.0f;
    }
    else if (h < 3.0f)
    {
      color.r = 0.0f;
      color.g = c;
      color.b = x;
    }
    else if (h < 4.0f)
    {
      color.r = 0.0f;
      color.g = x;
      color.b = c;
    }
    else if (h < 5.0f)
    {
      color.r = x;
      color.g = 0.0f;
      color.b = c;
    }
    else
    {
      color.r = c;
      color.g = 0.0f;
      color.b = x;
    }

    // Create a cube marker for the cluster volume
    visualization_msgs::Marker cube_marker;
    cube_marker.header.frame_id = cluster->header.frame_id;
    cube_marker.header.stamp = ros::Time::now();
    cube_marker.ns = "debug_clusters";
    cube_marker.id = i * 3;
    cube_marker.type = visualization_msgs::Marker::CUBE;
    cube_marker.action = visualization_msgs::Marker::ADD;

    // Set position (centroid)
    cube_marker.pose.position.x = centroid[0];
    cube_marker.pose.position.y = centroid[1];
    cube_marker.pose.position.z = centroid[2];
    cube_marker.pose.orientation.w = 1.0;

    // Set size
    cube_marker.scale.x = max_pt[0] - min_pt[0] + 0.01;
    cube_marker.scale.y = max_pt[1] - min_pt[1] + 0.01;
    cube_marker.scale.z = max_pt[2] - min_pt[2] + 0.01;

    // Set color
    cube_marker.color = color;

    // Create a point cloud marker to show the actual points
    visualization_msgs::Marker points_marker;
    points_marker.header = cube_marker.header;
    points_marker.ns = "debug_clusters";
    points_marker.id = i * 3 + 1;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.scale.x = 0.005; // Point size
    points_marker.scale.y = 0.005;
    points_marker.color = color;
    points_marker.color.a = 1.0; // Full opacity for points

    // Add individual points
    for (const auto &pt : cluster->points)
    {
      geometry_msgs::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      points_marker.points.push_back(p);
    }

    // Add text label with cluster ID and point count
    visualization_msgs::Marker text_marker;
    text_marker.header = cube_marker.header;
    text_marker.ns = "debug_clusters";
    text_marker.id = i * 3 + 2;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = centroid[0];
    text_marker.pose.position.y = centroid[1];
    text_marker.pose.position.z = max_pt[2] + 0.05; // Above the cluster
    text_marker.text = "Cluster " + std::to_string(i) + "\n" +
                       std::to_string(cluster->points.size()) + " pts";
    text_marker.scale.z = 0.03; // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    // Add all markers to array
    marker_array.markers.push_back(cube_marker);
    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(text_marker);
  }

  // Publish all markers
  marker_pub_.publish(marker_array);
  ROS_INFO("Published visualization markers for %lu debug clusters", clusters.size());
}

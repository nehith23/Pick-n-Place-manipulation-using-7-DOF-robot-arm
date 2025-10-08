#include "cw2_perception.h"

namespace cw2_perception
/**
 * @file cw2_perception.cpp
 * @brief Perception functions for detecting and classifying objects in the environment
 * 
 * This file contains utility functions for processing point clouds, detecting objects,
 * and classifying shapes (noughts and crosses) for the robot manipulation task.
 */
{
    // Shared static variable for debug image count
    static int debug_count = 1;
    static int orient_count = 1;

    // Convert cluster projection to binary image
    cv::Mat clusterToBinaryImage(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
        double resolution,
        bool save_debug)
    {
        if (cluster->empty())
        {
            return cv::Mat();
        }

        // Find the bounds of the cluster in X-Y
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        double max_y = -std::numeric_limits<double>::max();

        for (const auto &point : cluster->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;

            min_x = std::min(min_x, (double)point.x);
            min_y = std::min(min_y, (double)point.y);
            max_x = std::max(max_x, (double)point.x);
            max_y = std::max(max_y, (double)point.y);
        }

        // Calculate image dimensions with padding
        double padding = 0.02; // Add 2cm padding around the object
        int width = static_cast<int>((max_x - min_x + 2 * padding) / resolution) + 1;
        int height = static_cast<int>((max_y - min_y + 2 * padding) / resolution) + 1;

        if (width <= 0 || height <= 0 || width > 1000 || height > 1000)
        {
            ROS_WARN("clusterToBinaryImage: Invalid binary image dimensions %dx%d", width, height);
            return cv::Mat();
        }

        // Create blank image (white background)
        cv::Mat binaryImg = cv::Mat::zeros(height, width, CV_8UC1);

        // Project all points onto the image
        for (const auto &point : cluster->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;

            // Calculate image coordinates
            int img_x = static_cast<int>((point.x - min_x + padding) / resolution);
            int img_y = static_cast<int>((point.y - min_y + padding) / resolution);

            // Ensure within image bounds
            if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height)
            {
                // Mark the pixel as white (255)
                binaryImg.at<uchar>(img_y, img_x) = 255;
            }
        }

        // Apply morphological operations to clean the image and connect nearby points
        cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::dilate(binaryImg, binaryImg, structuringElement);
        cv::erode(binaryImg, binaryImg, structuringElement);

        // Flip the image vertically to match expected orientation
        cv::Mat flipped_img;
        cv::flip(binaryImg, flipped_img, 0); // 0 means flip around x-axis (vertically)

        // Then rotate 90° counterclockwise to fix orientation
        cv::Mat rotated_img;
        cv::rotate(flipped_img, rotated_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        // Save the image for debugging
        if (save_debug)
        {
            static int img_count = 0;
            std::string package_path = ros::package::getPath("cw2_team_7");
            std::string output_dir = package_path + "/src/images/binary/";
            boost::filesystem::create_directories(output_dir);
            // cv::imwrite(output_dir + "binary_projection_" + std::to_string(img_count++) + ".png", binaryImg);
            // cv::imwrite(output_dir + "binary_projection_" + std::to_string(img_count++) + ".png", flipped_img);
            cv::imwrite(output_dir + "binary_projection_" + std::to_string(img_count++) + ".png", rotated_img);
        }

        // return binaryImg;
        // return flipped_img;
        return rotated_img;
    }

    // Extract all clusters from a point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getAllClusters(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr,
        tf2_ros::Buffer &tfBuffer,
        double height_min,
        double height_max)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters;

        if (cloud_ptr->empty())
        {
            ROS_ERROR("getAllClusters: Input cloud is empty");
            return clusters;
        }

        // First, transform the cloud to the base frame "panda_link0"
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud =
            transformCloudToFrame(cloud_ptr, "panda_link0", tfBuffer);

        ROS_INFO("getAllClusters: Processing cloud with %lu points", transformed_cloud->points.size());

        // Filter out points near the robot base (cylindrical exclusion zone)
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr base_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // Define the exclusion zone parameters
        const double base_radius = 0.2; // 20cm radius
        const double base_center_x = 0.0;
        const double base_center_y = 0.0;

        // Copy points outside the exclusion zone
        for (const auto &point : transformed_cloud->points)
        {
            // Calculate distance from base center (ignoring z)
            double dx = point.x - base_center_x;
            double dy = point.y - base_center_y;
            double distance_from_center = std::sqrt(dx * dx + dy * dy);

            // Keep points outside the exclusion radius
            if (distance_from_center > base_radius)
            {
                base_filtered->points.push_back(point);
            }
        }

        base_filtered->width = base_filtered->points.size();
        base_filtered->height = 1;
        base_filtered->is_dense = false;
        base_filtered->header = transformed_cloud->header;

        ROS_INFO("getAllClusters: Filtered out robot base area, %lu points remaining", base_filtered->points.size());

        // Filter by X
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
        pass_x.setInputCloud(base_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-0.65, 0.75);
        pass_x.filter(*x_filtered);

        // Filter by Y
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xy_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PassThrough<pcl::PointXYZRGBA> pass_y;
        pass_y.setInputCloud(x_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.6, 0.6);
        pass_y.filter(*xy_filtered);

        // Now filter by Z height
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PassThrough<pcl::PointXYZRGBA> pass_z;
        pass_z.setInputCloud(xy_filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(height_min, height_max);
        pass_z.filter(*filtered_cloud);

        ROS_INFO("getAllClusters: After filtering, cloud has %lu points", filtered_cloud->points.size());

        // Optional: Downsample to reduce processing time
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
        voxel_grid.setInputCloud(filtered_cloud);
        voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f); // 5mm voxel size
        voxel_grid.filter(*downsampled_cloud);

        ROS_INFO("getAllClusters: After downsampling, cloud has %lu points", downsampled_cloud->points.size());

        // 2. Perform Euclidean clustering to separate objects
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(downsampled_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);    // Minimum points in a cluster
        ec.setMaxClusterSize(25000);  // Maximum points in a cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(downsampled_cloud);
        ec.extract(cluster_indices);

        ROS_INFO("getAllClusters: Found %lu initial clusters", cluster_indices.size());

        // 3. Extract each cluster as a separate point cloud
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

            for (const auto &index : indices.indices)
            {
                cluster->push_back(downsampled_cloud->points[index]);
            }

            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            cluster->header = downsampled_cloud->header;

            // Calculate the cluster size to filter out very small clusters
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            float x_size = max_pt[0] - min_pt[0];
            float y_size = max_pt[1] - min_pt[1];
            float z_size = max_pt[2] - min_pt[2];

            // Only keep clusters that meet minimum size requirements
            if (x_size > 0.01 && y_size > 0.01)
            { // At least 1cm in width/length
                clusters.push_back(cluster);
                ROS_INFO("Cluster with %lu points, size: [%.3f, %.3f, %.3f]",
                         cluster->size(), x_size, y_size, z_size);
            }
        }

        ROS_INFO("getAllClusters: Returning %lu valid clusters", clusters.size());
        return clusters;
    }

    // Get the orientation
    double getShapeOrientation(
        const cv::Mat &binaryImage,
        bool save_debug)
    {
        if (binaryImage.empty())
        {
            return 0.0;
        }

        // Create a working copy with preprocessing
        cv::Mat processedImg;

        // Ensure proper binary image with preprocessing
        if (binaryImage.type() != CV_8UC1)
        {
            cv::cvtColor(binaryImage, processedImg, cv::COLOR_BGR2GRAY);
        }
        else
        {
            processedImg = binaryImage.clone();
        }

        // Apply morphological operations to clean up the image (same as in isNought)
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(processedImg, processedImg, cv::MORPH_CLOSE, kernel);

        // Ensure binary with proper thresholding
        cv::threshold(processedImg, processedImg, 127, 255, cv::THRESH_BINARY);

        // Find contours after preprocessing
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processedImg.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // // Save preprocessed image if debug is enabled
        // if (save_debug)
        // {
        //     std::string package_path = ros::package::getPath("cw2_team_7");
        //     std::string output_dir = package_path + "/src/images/orientation/";
        //     boost::filesystem::create_directories(output_dir);
        //     static int orient_prep_count = 1;
        //     cv::imwrite(output_dir + "orientation_" + std::to_string(orient_prep_count++) + "_preprocessed.png", processedImg);
        // }

        if (contours.empty())
        {
            ROS_WARN("getShapeOrientation: No contours found");
            return 0.0;
        }

        // Find the largest contour
        int largest_idx = -1;
        double largest_area = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area)
            {
                largest_area = area;
                largest_idx = i;
            }
        }

        if (largest_idx < 0)
        {
            ROS_WARN("getShapeOrientation: No valid contour found");
            return 0.0;
        }

        // Fit a rotated rectangle
        cv::RotatedRect rect = cv::minAreaRect(contours[largest_idx]);

        // Get the angle
        double angle = rect.angle;

        // OpenCV returns angles in [-90,0], normalize to [0,180]
        if (angle < -45)
            angle += 90; // Rotate 90 degrees (equivalent orientation)

        // Flip the sign to match robot coordinate system
        angle = -angle;

        if (binaryImage.rows > binaryImage.cols)
        {
            // Additional 90-degree rotation may be needed based on how
            // projection from the point cloud to binary image
            angle += 90;
        }

        // In getShapeOrientation function, update the debug visualization part:

        if (save_debug)
        {
            cv::Mat debugImg;
            cv::cvtColor(binaryImage, debugImg, cv::COLOR_GRAY2BGR);

            // Draw the contour
            cv::drawContours(debugImg, contours, largest_idx, cv::Scalar(0, 255, 0), 2);

            // Draw the rotated rectangle
            cv::Point2f rect_points[4];
            rect.points(rect_points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(debugImg, rect_points[i], rect_points[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            }

            // Draw the orientation
            cv::Point2f center = rect.center;
            double length = std::max(rect.size.width, rect.size.height) * 0.25;

            // Convert to ROS convention for visualization
            double arrow_angle_rad = (-angle + 90) * CV_PI / 180.0;

            // Create the arrow endpoint
            cv::Point2f end(
                center.x - length * cos(arrow_angle_rad),
                center.y - length * sin(arrow_angle_rad));

            cv::arrowedLine(debugImg, center, end, cv::Scalar(255, 0, 0), 2);

            // Create a standardized banner with sufficient space
            int bannerHeight = 60;                          // Increased banner height
            int targetWidth = std::max(200, debugImg.cols); // Minimum width of 200px

            // Create a new image with standardized dimensions
            cv::Mat labelImg = cv::Mat(debugImg.rows + bannerHeight, targetWidth, CV_8UC3, cv::Scalar(255, 255, 255));

            // Calculate centering offset if original image is smaller than target width
            int xOffset = (targetWidth - debugImg.cols) / 2;

            // Copy the original image to the top part of the new image with proper centering
            debugImg.copyTo(labelImg(cv::Rect(xOffset, 0, debugImg.cols, debugImg.rows)));

            // Add text with angle information in the white banner area
            std::string angleText = "Angle: " + std::to_string(angle).substr(0, 6) + " degrees";

            // Calculate text size to center it horizontally
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(angleText, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            int textX = (targetWidth - textSize.width) / 2; // Center text horizontally

            // Draw text centered in the banner
            cv::putText(labelImg, angleText, cv::Point(textX, debugImg.rows + 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

            // Save the image with banner
            std::string package_path = ros::package::getPath("cw2_team_7");
            std::string output_dir = package_path + "/src/images/orientation/";
            boost::filesystem::create_directories(output_dir);
            // static int orient_count = 1;
            // cv::imwrite(output_dir + "orientation_" + std::to_string(orient_count++) + ".png", labelImg);
            cv::imwrite(output_dir + std::to_string(orient_count) + "_orientation" + ".png", labelImg);
        }

        orient_count++;

        ROS_INFO("Shape orientation: %.1f degrees", angle);
        return angle;
    }

    // Determine if a cluster is an obstacle (black object)
    bool isObstacle(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster)
    {
        if (cluster->empty())
        {
            return false;
        }

        // Calculate average RGB values
        uint32_t r_sum = 0, g_sum = 0, b_sum = 0;
        for (const auto &point : cluster->points)
        {
            r_sum += point.r;
            g_sum += point.g;
            b_sum += point.b;
        }

        uint8_t r_avg = r_sum / cluster->size();
        uint8_t g_avg = g_sum / cluster->size();
        uint8_t b_avg = b_sum / cluster->size();

        // Black obstacles have RGB values around [0.1, 0.1, 0.1] * 255 = [~26, ~26, ~26]
        // Allow some tolerance (values < 50 out of 255)
        const uint8_t threshold = 50;

        bool is_black = (r_avg < threshold && g_avg < threshold && b_avg < threshold);

        if (is_black)
        {
            ROS_INFO("isObstacle: Detected obstacle: RGB=(%d,%d,%d)", r_avg, g_avg, b_avg);
        }

        // Enhanced debugging
        ROS_INFO("Cluster RGB: (%d, %d, %d) - %s",
                 r_avg, g_avg, b_avg,
                 is_black ? "OBSTACLE" : "SHAPE");

        // Log height range for debugging
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        ROS_INFO("Cluster height: %.3f to %.3f meters", min_pt[2], max_pt[2]);

        return is_black;
    }

    // Add to cw2_perception.cpp
    bool isBasket(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
        double similarity_threshold)
    {
        if (cluster->empty())
        {
            return false;
        }

        // Target color for basket (brownish)
        const uint8_t target_r = 91;
        const uint8_t target_g = 37;
        const uint8_t target_b = 36;

        // Count points matching the color
        int matching_points = 0;
        int total_points = 0;

        // For calculating color statistics
        double mean_r = 0, mean_g = 0, mean_b = 0;

        // First pass - calculate mean color
        for (const auto &point : cluster->points)
        {
            // Skip invalid points
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }

            mean_r += point.r;
            mean_g += point.g;
            mean_b += point.b;
            total_points++;
        }

        // Calculate means
        if (total_points > 0)
        {
            mean_r /= total_points;
            mean_g /= total_points;
            mean_b /= total_points;
        }

        // Check if the mean color is close to the target color
        double r_diff = mean_r - target_r;
        double g_diff = mean_g - target_g;
        double b_diff = mean_b - target_b;

        // Calculate Euclidean distance in RGB space
        double color_distance = std::sqrt(r_diff * r_diff + g_diff * g_diff + b_diff * b_diff);

        // Maximum possible distance in RGB space is sqrt(3 * 255²)
        double max_distance = std::sqrt(3.0 * 255.0 * 255.0);
        double similarity = 1.0 - (color_distance / max_distance);

        ROS_DEBUG("Cluster color stats: R=%.1f, G=%.1f, B=%.1f, similarity=%.2f",
                  mean_r, mean_g, mean_b, similarity);

        // If similarity exceeds threshold, consider it a basket
        if (similarity > similarity_threshold)
        {
            ROS_INFO("Detected basket with color similarity %.2f (R=%.1f, G=%.1f, B=%.1f)",
                     similarity, mean_r, mean_g, mean_b);
            return true;
        }

        return false;
    }

    bool isNought(
        const cv::Mat &binaryImage,
        double hole_ratio_threshold)
    {
        if (binaryImage.empty())
        {
            return false;
        }

        // Create a working copy with preprocessing
        cv::Mat processedImg;

        // Ensure proper binary image with preprocessing
        if (binaryImage.type() != CV_8UC1)
        {
            cv::cvtColor(binaryImage, processedImg, cv::COLOR_BGR2GRAY);
        }
        else
        {
            processedImg = binaryImage.clone();
        }

        // Apply morphological operations to clean up the image
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(processedImg, processedImg, cv::MORPH_CLOSE, kernel);

        // Ensure binary with proper thresholding
        cv::threshold(processedImg, processedImg, 127, 255, cv::THRESH_BINARY);

        // Save the preprocessed image for debugging
        std::string package_path = ros::package::getPath("cw2_team_7");
        // static int debug_count = 1;
        std::string output_dir = package_path + "/src/images/classification/";
        boost::filesystem::create_directories(output_dir);

        cv::imwrite(output_dir + "cluster_" + std::to_string(debug_count) + "_a_preprocessed" + ".png", processedImg);

        // Find contours with hierarchy info for hole detection
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(processedImg.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            ROS_WARN("isNought: No contours found in binary image");
            return false;
        }

        // Create debug visualization
        cv::Mat debugImg = cv::Mat::zeros(processedImg.size(), CV_8UC3);

        // Find the largest contour (assumed to be the outer boundary)
        int largest_idx = -1;
        double largest_area = 0;

        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area)
            {
                largest_area = area;
                largest_idx = i;
            }
        }

        if (largest_idx < 0 || largest_area < 100)
        { // Area threshold to filter noise
            ROS_WARN("isNought: No sufficiently large contour found");
            return false;
        }

        // Draw the largest contour in green
        cv::drawContours(debugImg, contours, largest_idx, cv::Scalar(0, 255, 0), 2);
        cv::imwrite(output_dir + "cluster_" + std::to_string(debug_count) + "_b_outer_counter" + ".png", debugImg);

        // Create a mask of just the largest contour
        cv::Mat mask = cv::Mat::zeros(processedImg.size(), CV_8UC1);
        cv::drawContours(mask, contours, largest_idx, cv::Scalar(255), -1);
        // cv::imwrite(output_dir + "cluster_" + std::to_string(debug_count) + "_b2_mask" + ".png", mask);

        // Check for child contours (holes)
        double hole_area = 0;
        bool has_hole = false;

        for (int i = 0; i < contours.size(); i++)
        {
            // If this contour has the largest contour as a parent
            if (i != largest_idx && hierarchy[i][3] == largest_idx)
            {
                has_hole = true;
                double area = cv::contourArea(contours[i]);
                hole_area += area;
                cv::drawContours(debugImg, contours, i, cv::Scalar(0, 0, 255), 2); // Draw inner contours in red
            }
        }

        // Calculate hole ratio from contour hierarchy
        double contour_hole_ratio = hole_area / largest_area;

        // Decision
        bool is_nought = (contour_hole_ratio > hole_ratio_threshold);

        // Draw final visualization with stats
        cv::Mat finalDebug = debugImg.clone();

        // cv::putText(finalDebug, is_nought ? "NOUGHT" : "CROSS", cv::Point(15, 40),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, is_nought ? cv::Scalar(255, 255, 255) : cv::Scalar(255, 255, 255), 2);

        // cv::imwrite(output_dir + "cluster_" + std::to_string(debug_count) + "_c_final_" +
        //                 (is_nought ? "nought" : "cross") + ".png",
        //             finalDebug);

        // Create a standardized banner with sufficient space
        int bannerHeight = 60;                            // Increased banner height
        int targetWidth = std::max(200, finalDebug.cols); // Minimum width of 200px

        // Create a new image with standardized dimensions
        cv::Mat labelImg = cv::Mat(finalDebug.rows + bannerHeight, targetWidth, CV_8UC3, cv::Scalar(255, 255, 255));

        // Calculate centering offset if original image is smaller than target width
        int xOffset = (targetWidth - finalDebug.cols) / 2;

        // Copy the original image to the top part of the new image with proper centering
        finalDebug.copyTo(labelImg(cv::Rect(xOffset, 0, finalDebug.cols, finalDebug.rows)));

        // Add text with classification information in the white banner area
        std::string shapeText = is_nought ? "NOUGHT" : "CROSS";
        std::string ratioText = "Hole ratio: " + std::to_string(contour_hole_ratio).substr(0, 6);
        std::string fullText = shapeText + "   (" + ratioText + ")";

        // Calculate text size to center it horizontally
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(fullText, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
        int textX = (targetWidth - textSize.width) / 2; // Center text horizontally

        // Draw text centered in the banner
        cv::putText(labelImg, fullText, cv::Point(textX, finalDebug.rows + 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    is_nought ? cv::Scalar(0, 128, 0) : cv::Scalar(0, 0, 128), 1);

        // Save the image with banner
        cv::imwrite(output_dir + "cluster_" + std::to_string(debug_count) + "_c_final_" +
                        (is_nought ? "nought" : "cross") + ".png",
                    labelImg);

        debug_count++;

        ROS_INFO("Shape classification: hole_ratio=%.3f, is_nought=%s",
                 contour_hole_ratio, is_nought ? "true" : "false");

        return is_nought;
    }

    // Transform the point cloud to Panda Base frame
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformCloudToFrame(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in,
        const std::string &target_frame,
        tf2_ros::Buffer &tfBuffer)
    {
        if (cloud_in->empty())
        {
            return cloud_in;
        }

        // Get the source frame
        std::string source_frame = cloud_in->header.frame_id;

        if (source_frame == target_frame)
        {
            // No transformation needed
            return cloud_in;
        }

        // Create a new point cloud for the transformed points
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>());

        try
        {
            // Lookup the transform from source to target frame
            geometry_msgs::TransformStamped transform =
                tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));

            // Convert TF2 transform to an Eigen transformation matrix
            Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();
            tf2::Transform tf2_transform;
            tf2::fromMsg(transform.transform, tf2_transform);

            Eigen::Quaterniond q(
                tf2_transform.getRotation().getW(),
                tf2_transform.getRotation().getX(),
                tf2_transform.getRotation().getY(),
                tf2_transform.getRotation().getZ());

            eigen_transform.translate(Eigen::Vector3d(
                tf2_transform.getOrigin().getX(),
                tf2_transform.getOrigin().getY(),
                tf2_transform.getOrigin().getZ()));

            eigen_transform.rotate(q);

            // Transform the point cloud
            pcl::transformPointCloud(*cloud_in, *cloud_out, eigen_transform.cast<float>());

            // Set the new frame ID
            cloud_out->header.frame_id = target_frame;

            ROS_INFO("transformCloudToFrame: Transformed point cloud from %s to %s",
                     source_frame.c_str(), target_frame.c_str());

            return cloud_out;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("transformCloudToFrame: Transform failed: %s", ex.what());
            return cloud_in; // Return original if transform fails
        }
    }

    // Clear Debug images
    void clearDebugImages()
    {
        // Get package path
        std::string package_path = ros::package::getPath("cw2_team_7");

        // Define directories to clear
        std::vector<std::string> dirs_to_clear = {
            "/src/images/binary/",
            "/src/images/classification/",
            "/src/images/orientation/"};

        // Delete files in each directory
        for (const auto &dir : dirs_to_clear)
        {
            std::string dir_path = package_path + dir;

            try
            {
                // Check if directory exists first
                if (boost::filesystem::exists(dir_path))
                {
                    ROS_INFO("Clearing debug images from %s", dir_path.c_str());

                    // Iterate through directory entries
                    for (boost::filesystem::directory_iterator end_dir_it, it(dir_path);
                         it != end_dir_it; ++it)
                    {
                        try
                        {
                            // Remove only regular files (not directories)
                            if (boost::filesystem::is_regular_file(it->path()))
                            {
                                boost::filesystem::remove(it->path());
                            }
                        }
                        catch (const boost::filesystem::filesystem_error &e)
                        {
                            ROS_WARN("Failed to remove file %s: %s",
                                     it->path().string().c_str(), e.what());
                        }
                    }
                }
                else
                {
                    // Create the directory if it doesn't exist
                    boost::filesystem::create_directories(dir_path);
                    ROS_INFO("Created directory %s", dir_path.c_str());
                }
            }
            catch (const boost::filesystem::filesystem_error &e)
            {
                ROS_ERROR("Error cleaning directory %s: %s", dir_path.c_str(), e.what());
            }
        }

        // Reset debug counters for images
        debug_count = 1; // Reset the static counter
        orient_count = 1;
    }

    // Detect table height from point cloud
    double detectTableHeight(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                             tf2_ros::Buffer &tfBuffer)
    {
        if (cloud->empty())
        {
            ROS_ERROR("detectTableHeight: Input cloud is empty!");
            return 0.0;
        }

        // First, transform the cloud to the base frame "panda_link0"
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud =
            transformCloudToFrame(cloud, "panda_link0", tfBuffer);

        if (transformed_cloud->empty())
        {
            ROS_ERROR("detectTableHeight: Failed to transform cloud to panda_link0 frame!");
            return 0.0;
        }

        ROS_INFO("detectTableHeight: Transformed cloud has %lu points (frame: %s)",
                 transformed_cloud->points.size(), transformed_cloud->header.frame_id.c_str());

        // Sample and log first few points' RGB values
        ROS_INFO("detectTableHeight: Sampling first 5 points RGB values:");
        for (int i = 0; i < std::min(5, (int)transformed_cloud->points.size()); i++)
        {
            const auto &p = transformed_cloud->points[i];
            ROS_INFO("  Point %d: RGB=(%d, %d, %d)", i, p.r, p.g, p.b);
        }

        // Find maximum values to understand the color range
        uint8_t max_r = 0, max_g = 0, max_b = 0;
        for (const auto &p : transformed_cloud->points)
        {
            max_r = std::max(max_r, p.r);
            max_g = std::max(max_g, p.g);
            max_b = std::max(max_b, p.b);
        }
        ROS_INFO("detectTableHeight: Max RGB values: (%d, %d, %d)", max_r, max_g, max_b);

        // Create point cloud of table points using green dominance approach
        std::vector<float> table_heights;
        table_heights.reserve(1000);

        // Target RGB values for the green table (observed from logs)
        const uint8_t TARGET_R = 168;
        const uint8_t TARGET_G = 198;
        const uint8_t TARGET_B = 168;

        // Acceptable color distance threshold
        const double COLOR_THRESHOLD = 40.0; // Euclidean distance in RGB space

        // Scan a limited portion of points for efficiency
        int step = std::max(1, static_cast<int>(transformed_cloud->points.size() / 10000));
        int points_checked = 0;
        int matching_points = 0;

        // Try two strategies:

        // Strategy 1: Find points matching target color
        for (size_t i = 0; i < transformed_cloud->points.size(); i += step)
        {
            const auto &point = transformed_cloud->points[i];
            points_checked++;

            // Calculate color distance using Euclidean distance in RGB space
            double r_diff = point.r - TARGET_R;
            double g_diff = point.g - TARGET_G;
            double b_diff = point.b - TARGET_B;
            double color_distance = std::sqrt(r_diff * r_diff + g_diff * g_diff + b_diff * b_diff);

            // If color is close to target, record the z value
            if (color_distance < COLOR_THRESHOLD)
            {
                table_heights.push_back(point.z);
                matching_points++;
            }
        }

        ROS_INFO("detectTableHeight: Checked %d points, found %d matching table color",
                 points_checked, matching_points);

        // Strategy 2: If first strategy didn't work well, try green dominance
        if (table_heights.size() < 100)
        {
            ROS_WARN("Few color matches found, trying green dominance approach...");
            table_heights.clear();

            for (size_t i = 0; i < transformed_cloud->points.size(); i += step)
            {
                const auto &point = transformed_cloud->points[i];

                // Green dominance check (green significantly higher than red/blue)
                if (point.g > point.r + 15 && point.g > point.b + 15 && point.g > 135)
                {
                    table_heights.push_back(point.z);
                }
            }

            ROS_INFO("detectTableHeight: Found %lu points using green dominance", table_heights.size());
        }

        if (table_heights.empty())
        {
            ROS_WARN("No table points detected using either method!");
            return 0.0;
        }

        // Sort heights to find median (more robust than mean with outliers)
        std::sort(table_heights.begin(), table_heights.end());
        double median_height = table_heights[table_heights.size() / 2];

        // Calculate the 10th percentile as the likely table surface
        size_t percentile_10_idx = std::min((size_t)(table_heights.size() * 0.1), table_heights.size() - 1);
        double percentile_10_height = table_heights[percentile_10_idx];

        ROS_INFO("Table heights - min: %.3f, 10th percentile: %.3f, median: %.3f, max: %.3f (from %lu points)",
                 table_heights.front(), percentile_10_height, median_height, table_heights.back(),
                 table_heights.size());

        // Use the 10th percentile as our result (avoids outliers while getting close to the actual surface)
        return percentile_10_height;
    }
}
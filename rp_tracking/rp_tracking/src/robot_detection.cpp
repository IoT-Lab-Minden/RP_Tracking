#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <eigen_conversions/eigen_msg.h>

#include <deque>
#include <array>

#include "region_growing.h"
#include <computeCentroidAndOBB.hpp>

// Error weights
const double WEIGHT_ASPECT_RATIO = 3;
const double WEIGHT_VOLUME_DIFF = 1;
const double WEIGHT_DISTANCE = 3;

// Maximum allowed model errors, no best continuous cluster is selected,
// if the error is above this value
const double MAX_MODEL_ERROR = 0.5;
// Maximum continuous cluster error for a new cluster to be considered
// part of a continuous cluster
const double MAX_CONT_ERROR = 0.5; //0.3;
// Maximum number of model errors stored in the model_errors list of a continuous cluster
const ulong MAX_ERROR_COUNT = 64;
// Maximum duration after which a cluster with no updates is removed
const double CLUSTER_KEEP_DURATION = 5.0; // in secs
// Required point count for new continuous clusters and continuous cluster updates, that
// are not the best continuous cluster
const int MIN_CLUSTER_POINT_COUNT = 8;
// All bounding box lenghts are increased to at least this value
const double MIN_DIMENSION_LENGTH = 0.05;
// Maximum distance for tracking the best cluster
const double MAX_TRACK_DISTANCE = 0.2;
// Time, after which a continuous cluster is tested for validity
const double MIN_DETECTION_TIME = 2.0;
// Max size of the message time array
const int MAX_MESSAGE_TIME_COUNT = 20;

/**
 * @struct ClusterError
 * @brief Contains a summery of the cluster errors
 */
struct ClusterError {
    double aspect_ratio; // error of the aspect ratio difference
    double volume_diff; // error of the volume difference
    double distance; // euclidean distance error
    double overall; // All three errors combined
    double overall_static; // Aspect ration and volume diff error combined
};

/**
 * @struct ContinuousCluster
 * @brief Stores a history of clusters, that are from the same tracked object
 */
struct ContinuousCluster {
    std::deque<ClusterError> model_errors; // List of errors between a cluster of this continuous cluster and the model
    ClusterError last_cont_error; // Error between the last and current cluster
    jsk_recognition_msgs::BoundingBox last_bb; // Bounding box of the last/current cluster
    ros::Time first_occurence_time; // Timestamp, the first single cluster appeared
    ros::Time last_update_time; // Last time, this continuous cluster was updated either as part of the foreground or background
    ros::Time last_foreground_time; // Last time, this continuous cluster was updated as part of the foreground
    ulong size; // Number of clusters, that have been assigned to this continuous cluster
    bool valid; // If this continuous cluster has passed the validity check
};

typedef pcl::PointXYZ PointType;

/**
 * @class RobotDetection
 * @brief Based on previous cluster detection, detects and tracks robotic platforms based on a simple model.
 */
class RobotDetection {
    private:
        ros::NodeHandle nh;
        message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_cluster_obb;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_no_ground_cloud;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_original_cloud;
        typedef message_filters::sync_policies::ExactTime<jsk_recognition_msgs::BoundingBoxArray, sensor_msgs::PointCloud2> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
        ros::Publisher pub_obb;
        ros::Publisher pub_pose;

        std::string topic_obb;
        std::string topic_no_ground_cloud;

        jsk_recognition_msgs::BoundingBox robot_model; // Robot dimensions

        std::vector<ContinuousCluster> past_clusters; // List containing all continuous clusters
        int last_best_cluster_index = -1; // Index of the best cluster in the last frame

        std::deque<ros::Time> message_times; // List of the last frame timestamps

    public:
        /**
         * @brief Reads parameters and creates subscribers and publishers
         * 
         * The subscribers for the clusters and the no_ground_cloud are synchronized
         */
        RobotDetection() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            sub_cluster_obb.subscribe(nh, topic_obb, 100);
            sub_no_ground_cloud.subscribe(nh, topic_no_ground_cloud, 100);
            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), sub_cluster_obb, sub_no_ground_cloud);
            sync->registerCallback(boost::bind(&RobotDetection::syncedCallback, this, _1, _2));
            pub_obb = nh.advertise<jsk_recognition_msgs::BoundingBox>("/best_robot_obb", 1);
            pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_detect", 1);
            
            ROS_INFO("Using model with dimensions %f %f %f", robot_model.dimensions.x,
                robot_model.dimensions.y, robot_model.dimensions.z);
        }

        /**
         * @brief Callback function for synchronized cluster and no_ground_cloud subscribers
         * 
         * @param cluster_obb_msg Message containing all currently detected clusters
         * @param no_ground_cloud Cloud after processed by GroundRemoval node
         */
        void syncedCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& cluster_obb_msg, const sensor_msgs::PointCloud2::ConstPtr& no_ground_cloud_msg) {
            ros::Time now = cluster_obb_msg->header.stamp;

            // Resetting robot detection, when looping rosbags
            if ((now - getLastUpdateTime()).toNSec() < 0) {
                ROS_WARN("Detected timestamp from the past. Resetting.");
                resetRobotDetection();
            }
            // Storing timing information, required for continuous cluster validity chekc
            updateMessageTime(now);
            ROS_INFO("FPS: %f", getUpdateRate());

            // Updates continuous cluster using the newly received clusters information
            updatePastClusters(cluster_obb_msg, now);

            // Track current best cluster, if it was not updated through cluster detection
            trackBestNonDetectedCluster(last_best_cluster_index, now, no_ground_cloud_msg, no_ground_cloud_msg->header);

            // Update the remaining continuous cluster data, that could not have been updated
            // before all newly received clusters have been processed and the best cluster has
            // been tracked.
            fixContinuousClusterData(now);
            // Removes all continuous clusters, that have not been updated for a longer time
            removeOldClusters(now);

            // Determine best cluster
            int best_cluster_index = findBestContinuousCluster(now);
            jsk_recognition_msgs::BoundingBox refined_bb;
            // Refine the pose using the finer (not downsampled) point cloud
            if (refineBestClusterPose(best_cluster_index, no_ground_cloud_msg, now, refined_bb)) {
                // And publish the pose
                publishBestClusterPose(refined_bb, cluster_obb_msg->header, now);
                last_best_cluster_index = best_cluster_index;
            }
        }

        ros::Time getLastUpdateTime() {
            if (message_times.size() > 0) {
                return message_times.at(message_times.size()-1);
            } else {
                return ros::Time(0);
            }
        }

        /**
         * @brief Add current timestamp to update time array
         * 
         * Ensures that the number of timestamps does not exceed a limit.
         * 
         * @param stamp Current timestamp
         */
        void updateMessageTime(ros::Time& stamp) {
            message_times.push_back(stamp);
            while (message_times.size() > MAX_MESSAGE_TIME_COUNT) {
                message_times.pop_front();
            }
        }

        /**
         * @brief Calculates the update rate, at which messages are received
         * 
         * @return update rate in updates per second
         */
        float getUpdateRate() {
            if (message_times.size() <= 1) return 0.0;

            float diff_sum;
            for (int i = 1; i < message_times.size(); i++) {
                float diff = (message_times.at(i) - message_times.at(i-1)).toSec();
                diff_sum += diff;
            }
            float update_time = diff_sum / (message_times.size()-1);
            float update_rate = 1 / update_time;
            return update_rate;
        }

        /**
         * @brief Checks, if the given continuous cluster is valid
         * 
         * Checks for validity of a given continuous cluster and sets its corresponding
         * data attribute. A continuous cluster is valid, if it has been detected for
         * at least MIN_DETECTION_TIME and has been detected in every frame except one.
         * 
         * @param cluster_id Index of the continuous cluster in the class array
         */
        bool isValidContinuousCluster(int cluster_id) {
            ContinuousCluster cluster = past_clusters.at(cluster_id);

            // Return true, if it has already been declared as valid
            if (cluster.valid) return true;

            // Return false, if the cluster has not been detected for long enough
            if ((cluster.last_foreground_time - cluster.first_occurence_time).toSec() < MIN_DETECTION_TIME)
                return false;

            float update_rate = getUpdateRate();

            // Since we don't want to classify clusters with more than one missing continuous detection as valid,
            // we check that the difference between last_foreground_time and first_occurence_time is not too large
            // Note: We multiply the inverse of the update rate by two to account for the one missing detection and
            // for possible delay.
            if ((cluster.last_foreground_time - cluster.first_occurence_time).toSec() > MIN_DETECTION_TIME + 2*(1/update_rate))
                return false;

            // Calculate number of required detection
            int update_count = std::ceil(update_rate * MIN_DETECTION_TIME)-1;
            if (update_count < 0) return false; // Should never happen as this is only returned when no information is available
            // and that is only the case for the first frame

            // Check if it has reached the required update count
            if (cluster.size < update_count) return false;

            // Set it to valid, so we no longer have to check it
            past_clusters.at(cluster_id).valid = true;
            return true;
        }

        /**
         * @brief Resets everything
         */
        void resetRobotDetection() {
            past_clusters.clear();
            last_best_cluster_index = -1;
            message_times.clear();
        }

        /**
         * @brief Publish pose of the tracked cluster
         * 
         * @param refined_bb Current bounding box of the cluster, including the position and orientation
         * @param header For tf frame information
         * @param stamp Timestamp for the current bounding box
         */
        void publishBestClusterPose(jsk_recognition_msgs::BoundingBox& refined_bb, std_msgs::Header header, ros::Time stamp) {
            refined_bb.header = header;
            refined_bb.header.stamp = stamp;
            refined_bb.dimensions.x = std::max(refined_bb.dimensions.x, MIN_DIMENSION_LENGTH);
            refined_bb.dimensions.y = std::max(refined_bb.dimensions.y, MIN_DIMENSION_LENGTH);
            refined_bb.dimensions.z = std::max(refined_bb.dimensions.z, MIN_DIMENSION_LENGTH);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header = header;
            pose_msg.header.stamp = stamp;
            pose_msg.pose = refined_bb.pose;

            pub_pose.publish(pose_msg);
            pub_obb.publish(refined_bb);
        }

        /**
         * @brief Extracts a cluster from a point cloud based on its bounding box
         * 
         * @param input_cloud full cloud
         * @param output_cloud containing only the cluster
         * @param pose of the cluster in the input cloud
         * @param offset to ensure that we get all points even if the cluster has moved
         *               or there are inaccuracies
         */
        void cropCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, jsk_recognition_msgs::BoundingBox& bb, float offset) {
            // Convert pose data
            pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(input_cloud);
            Eigen::Quaterniond quat_eigen;
            tf::quaternionMsgToEigen(bb.pose.orientation, quat_eigen);
            Eigen::Vector3f rotation = quat_eigen.toRotationMatrix().eulerAngles(0, 1, 2).cast<float>();
            crop_box.setRotation(rotation);
            Eigen::Vector3d position_eigen;
            tf::pointMsgToEigen(bb.pose.position, position_eigen);

            // Extract rough surrounding of the cluster
            crop_box.setTranslation(position_eigen.cast<float>());
            crop_box.setMin(Eigen::Vector4f((-1 * bb.dimensions.x/2)-offset, (-1 * bb.dimensions.y/2)-offset, (-1 * bb.dimensions.z/2)-offset, 1.0));
            crop_box.setMax(Eigen::Vector4f((bb.dimensions.x/2)+offset, (bb.dimensions.y/2)+offset, (bb.dimensions.z/2)+offset, 1.0));
            crop_box.filter(*output_cloud);
        }

        /**
         * @brief Extracts a cluster from a given input cloud
         * 
         * @param input_cloud
         * @param output_cloud cluster
         * @param pose of the cluster in the input cloud
         * 
         * @return true, if extracting was successful
         */
        bool extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, geometry_msgs::Pose& pose) {

            // Find the closest point of the cluster near the bounding box center
            // Required to initialize region growing
            pcl::shared_ptr<pcl::Indices> neighbours(new pcl::Indices());
            std::vector<float> distances;
            pcl::search::KdTree<PointType> kdtree;
            kdtree.setInputCloud(input_cloud);
            PointType point(pose.position.x, pose.position.y, pose.position.z);
            kdtree.nearestKSearch(point, 1, *neighbours, distances);

            // Return, if no point close to the center was found
            // In this case, the tracked object has moved too much (which should have
            // been covered by cluster detection) or a part of another object is part
            // of the cropped area
            if (distances[0] > MAX_TRACK_DISTANCE) {
                ROS_INFO("Distance to cloud point too large");
                return false;
            }

            // Extract cluster using the cropped and downsampled cloud
            RegionGrowing<PointType> object_segmentation;
            object_segmentation.setMinClusterSize (1);
            object_segmentation.setMaxClusterCount (1);
            object_segmentation.setForegroundBackgroundRatio(0.0);
            object_segmentation.setSearchRadius (0.2);
            object_segmentation.setInputCloud(input_cloud);
            object_segmentation.setIndices(neighbours);

            std::vector<pcl::PointIndices> clusters;
            object_segmentation.extract (clusters);

            // No cluster was found, will never happen
            if (clusters.size() <= 0) {
                ROS_INFO("No cluster found");
                return false;
            }

            pcl::PointIndices::Ptr cluster(new pcl::PointIndices);
            cluster->indices = clusters[0].indices;

            // Same as above
            if (cluster->indices.size() == 0) {
                ROS_INFO("Only empty cluster found");
                return false;
            }

            // Extract cluster cloud
            pcl::ExtractIndices<PointType> extractor;
            pcl::PointCloud<PointType>::Ptr new_cluster (new pcl::PointCloud<PointType>);
            extractor.setInputCloud(input_cloud);
            extractor.setIndices(cluster);
            extractor.setNegative(false);
            extractor.filter(*output_cloud);

            return true;
        }

        /**
         * @brief Converts point cloud cluster into bounding box
         * 
         * @param cluster_cloud
         * @param bb bounding box message output
         */
        void clusterToBbMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud, jsk_recognition_msgs::BoundingBox& bb) {
            // Calculate oriented bounding box
            Eigen::Matrix<double, 3, 1> centroid, obb_center, obb_dimensions;
            Eigen::Matrix<double, 3, 3> obb_rotational_matrix;
            computeCentroidAndOBB(*cluster_cloud, centroid, obb_center, obb_dimensions, obb_rotational_matrix);

            // Fill bounding box data structure
            tf::pointEigenToMsg(obb_center, bb.pose.position);
            Eigen::Quaterniond eigen_quat(obb_rotational_matrix);
            tf::quaternionEigenToMsg(eigen_quat, bb.pose.orientation);
            tf::vectorEigenToMsg(obb_dimensions, bb.dimensions);
            bb.label = 0;
            bb.value = cluster_cloud->size();
        }

        /**
         * @brief Inproves the accuracy of the bounding box using the no_ground_cloud
         * 
         * @param best_cluster_index Index to the cluster in the contiunous cluster array
         * @param no_ground_cloud_msg Message as received by the callback function
         * @param now Current timestamp
         * @param refined_bb Out parameter for the refined bounding box
         * 
         * @return true, if refinement was successful
         */
        bool refineBestClusterPose(int best_cluster_index, const sensor_msgs::PointCloud2::ConstPtr& no_ground_cloud_msg, ros::Time now, jsk_recognition_msgs::BoundingBox& refined_bb) {
            // Don't do anything, if best_cluster_index is not valid
            if (best_cluster_index < 0) return false;

            // Convert point cloud message
            pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*no_ground_cloud_msg, *no_ground_cloud);

            jsk_recognition_msgs::BoundingBox bb = past_clusters.at(best_cluster_index).last_bb;

            // Extract rough surrounding of the cluster
            pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cropCluster(no_ground_cloud, cropped_cloud, bb, 0.05);

            // Extract new cluster of the object for refinement
            pcl::PointCloud<PointType>::Ptr cluster_cloud (new pcl::PointCloud<PointType>);
            extractCluster(no_ground_cloud, cluster_cloud, bb.pose);
            
            // Convert point cloud cluster to bounding box
            clusterToBbMsg(cluster_cloud, refined_bb);

            return true;
        }

        /**
         * @brief Tracks a continuous cluster that has not been detected through cluster detection in this timestep
         * 
         * Searches for a cluster that has not been detected through cluster detection at the current position of
         * the continuous cluster. This is required, when the tracked object has not moved over a certain time
         * period and has therefore faded into the background. In that case, the BackgroundRemoval does not include
         * the cluster in the foreground cloud and the ClusterDetection is unable to remove it.
         * 
         * @param cluster_id Index of the best continuous cluster in the continuous cluster array
         * @param no_ground_cloud_msg Message as received by the callback function
         * @param header Header message of the current timestep
         * 
         * @return true, if the cluster was updated
         */
        bool trackBestNonDetectedCluster(int cluster_id, ros::Time now, const sensor_msgs::PointCloud2::ConstPtr& no_ground_cloud_msg, std_msgs::Header header) {
            if (cluster_id >= past_clusters.size() || cluster_id < 0) {
                ROS_WARN("Tried to access invalid cluster id %d", cluster_id);
                return false;
            }

            // When the cluster has not been updated in the current step, we assume that it needs tracking
            if (past_clusters.at(cluster_id).last_update_time != now) {
                pcl::PointCloud<PointType>::Ptr no_ground_cloud (new pcl::PointCloud<PointType>);
                pcl::fromROSMsg(*no_ground_cloud_msg, *no_ground_cloud);

                // Get last known bb
                jsk_recognition_msgs::BoundingBox bb = past_clusters.at(cluster_id).last_bb;

                // Extract rough surrounding of the cluster
                // Enlarge the CropBox by 0.25 to counteract voxel inaccuracy(0.05) and non-detected movement(0.2)
                pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                cropCluster(no_ground_cloud, cropped_cloud, bb, 0.25);

                // Downsample point cloud as it is done in cluster detection
                pcl::VoxelGrid<PointType> voxelGrid;
                voxelGrid.setInputCloud(cropped_cloud);
                float leaf_size = 0.05;
                voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
                pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                voxelGrid.filter(*voxelized_cloud);

                // Return, if no points were found
                // This can happen, when the tracked object is moving outside the visible area
                if (voxelized_cloud->size() == 0) {
                    ROS_INFO("Tracking: Empty crop cloud");
                    return false;
                }

                // Extract new cluster of the object for tracking
                pcl::PointCloud<PointType>::Ptr new_cluster (new pcl::PointCloud<PointType>);
                bool success = extractCluster(no_ground_cloud, new_cluster, bb.pose);
                if(!success) {
                    ROS_INFO("Tracking: Failed to extract cluster");
                    return false;
                }

                // Convert point cloud cluster to bounding box
                jsk_recognition_msgs::BoundingBox jsk_obb;
                clusterToBbMsg(new_cluster, jsk_obb);

                // Calculate errors
                geometry_msgs::Point::ConstPtr position_ptr(new geometry_msgs::Point(past_clusters.at(cluster_id).last_bb.pose.position));
                ClusterError error = calculateError(jsk_obb, position_ptr, past_clusters.at(cluster_id).last_bb);
                ClusterError model_error = calculateError(jsk_obb, nullptr, robot_model);

                // Update continuous cluster
                past_clusters.at(cluster_id).last_bb = jsk_obb;
                past_clusters.at(cluster_id).last_cont_error = error;
                past_clusters.at(cluster_id).last_update_time = now;
                
                return true;
            }
            // We also return true here, since there was no tracking required
            return true;
        }

        /**
         * @brief Find the continuous cluster with the smallest model error
         * 
         * For every valid continuous cluster, this method calculates the mean
         * static overall modell error and finds the continuous cluster with
         * the smallest mean error.
         * 
         * @param time current timestamp
         * 
         * @return Array index of the best continuous cluster
         */
        int findBestContinuousCluster(ros::Time now) {
            int best_index = -1;
            double best_avg_error = std::numeric_limits<double>::max();

            // For every continuous cluster
            for (int i = 0; i < past_clusters.size(); i++) {
                // Only calculate the error for a valid continuous cluster
                if (!isValidContinuousCluster(i)) continue;

                // Calculate the mean static overall model error
                double error_sum = 0.0;
                for (ClusterError error : past_clusters.at(i).model_errors) {
                    error_sum += error.overall_static;
                }
                double avg_error = error_sum / past_clusters.at(i).model_errors.size();

                // Update best error
                if (avg_error < best_avg_error) {
                    best_index = i;
                    best_avg_error = avg_error;
                }
            }

            // Reset index, if we did not receive an update of the best continuous cluster in this frame
            if (best_index >= 0 && past_clusters.at(best_index).last_update_time < now) best_index = -1;

            // Ignore best mean error, if it doesn't reach a threshold
            if (best_avg_error > MAX_MODEL_ERROR) best_index = -1;
            
            if (best_index >= 0)
                ROS_INFO("Best continuous cluster error %f", best_avg_error);
            else
                ROS_WARN("Failed to determine best cluster, best error %f", best_avg_error);

            return best_index;
        }

        /**
         * @brief Creates a new continuous cluster
         * 
         * @param bb Bounding box of the first cluster
         * @param now Current timestamp
         */
        void insertNewCluster(jsk_recognition_msgs::BoundingBox bb, ros::Time now) {
            // Require a certain number of points, therefore also ignore very sparse information clusters
            if (bb.value < MIN_CLUSTER_POINT_COUNT) return;

            // Initialize continuous cluster object
            ContinuousCluster ccluster;
            ccluster.model_errors.push_back(calculateError(bb, nullptr, robot_model));
            ccluster.last_bb = bb;
            ccluster.last_cont_error = ClusterError();
            ccluster.first_occurence_time = now;
            ccluster.last_update_time = now;
            ccluster.last_foreground_time = now;
            ccluster.size = 0;
            ccluster.valid = false;
            // And add it to the list
            past_clusters.push_back(ccluster);
        }

        /**
         * @brief Updates all continuous cluster based on new cluster information
         * 
         * Tries to add all newly received clusters to existing continuous clusters. If
         * no fitting continuous cluster was found, a new continuous cluster is added for
         * the given cluster.
         * 
         * @param cluster_obb_msg Newly received clusters
         * @param now Current timestamp
         */
        void updatePastClusters(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& cluster_obb_msg, ros::Time now) {
            int bb_counter = 0;
            // For every newly received cluster
            for (jsk_recognition_msgs::BoundingBox bb : cluster_obb_msg->boxes) {
                // Try to find a fitting continuous cluster
                bool isContinuous = updateSinglePastCluster(bb, now);
                
                // If no continuous cluster was found, create a new one based on
                // the current bounding box
                if (!isContinuous) {
                    insertNewCluster(bb, now);
                }

                bb_counter++;
            }
            ROS_INFO("Received %d clusters, num continuous clusters: %lu", bb_counter, past_clusters.size());
        }

        /**
         * @brief Insert a given cluster into the best fitting continuous cluster
         * 
         * @param bb Bounding box of the new cluster
         * @param now Current timestamp
         * 
         * @return true, when a fitting continuous cluster was found
         */
        bool updateSinglePastCluster(jsk_recognition_msgs::BoundingBox bb, ros::Time now) {
            int best_index = -1;
            ClusterError error, best_error;
            best_error.overall = MAX_CONT_ERROR;

            // Determine the best fitting continuous cluster
            for (int i = 0; i < past_clusters.size(); i++) {
                geometry_msgs::Point::ConstPtr position_ptr(new geometry_msgs::Point(past_clusters.at(i).last_bb.pose.position));
                // Calculate the error between the new cluster and the current cluster of the continuous cluster
                error = calculateError(bb, position_ptr, past_clusters.at(i).last_bb);

                if (error.overall < best_error.overall) {
                    best_index = i;
                    best_error = error;
                }
            }

            // Only add low point count clusters to the best continuous cluster,
            // otherwise the algorithm can become unstable as the correspondence
            // is only determined by the distance error.
            if (bb.value < MIN_CLUSTER_POINT_COUNT && best_index != last_best_cluster_index) return false;

            // If a fitting cluster was found
            if (best_index >= 0) {
                // And it has not received an update in this timestep
                if (past_clusters.at(best_index).last_update_time != now) {
                    // Then we can just add the new cluster information
                    // Note: If a continuous cluster has no actual cluster in this frame, a wrong cluster that
                    // has no corresponding continuous cluster and therefore would normally form a new cluster
                    // could be inserted here instead. This would merge two non-corresponding continuous cluster.
                    // The chance for this error is reduced by using a MAX_CONT_ERROR.
                    past_clusters.at(best_index).last_bb = bb;
                    past_clusters.at(best_index).last_cont_error = best_error;
                    
                    past_clusters.at(best_index).last_update_time = now;
                    past_clusters.at(best_index).last_foreground_time = now;
                } else if (past_clusters.at(best_index).last_cont_error.overall > error.overall) {
                    // Otherwise, if this cluster has a lower error than the other cluster, that was added in
                    // this timestep, we need to replace the possbily falsely added cluster.

                    // Save old bounding box, because we want to readd it somewhere else after updating this 
                    // continuous cluster
                    jsk_recognition_msgs::BoundingBox old = past_clusters.at(best_index).last_bb;
                    past_clusters.at(best_index).last_bb = bb;
                    past_clusters.at(best_index).last_cont_error = best_error;
                    
                    past_clusters.at(best_index).last_update_time = now;
                    past_clusters.at(best_index).last_foreground_time = now;

                    // Try to insert falsely inserted cluster into a different continuous cluster
                    if (!updateSinglePastCluster(old, now)) {
                        // Insert a new cluster, if unable to find a different fitting cluster
                        insertNewCluster(old, now);
                    }
                }
                // TODO: Bug: We are missing an else case where we need to determine the next best
                // option of the current cluster. Now, we are just throwing it away. As this is the
                // evaluated code, we are not fixing this bug now.
                // Steps to fix:
                //     1. Store a sorted list of the best indices instead of just the best one
                //     2. Iterate over this list starting at the best index and stop, when the cluster
                //        was inserted in a continuous cluster
                return true;
            }
            return false;
        }

        /**
         * @brief Updates the continuous cluster data for this timestep
         * 
         * Some data can only be updated after all new clusters are inserted
         * and the best continuous cluster was tracked.
         * 
         * @param now Current timestep
         */
        void fixContinuousClusterData(ros::Time now) {
            for (int i = 0; i < past_clusters.size(); i++) {
                // Only fix clusters, that have been updated at this timestep
                if (past_clusters.at(i).last_update_time == now) {
                    // Increase the cluster count
                    past_clusters.at(i).size++;

                    // Recalculate the model error
                    if (past_clusters.at(i).last_bb.value > MIN_CLUSTER_POINT_COUNT) {
                        past_clusters.at(i).model_errors.push_back(calculateError(past_clusters.at(i).last_bb, nullptr, robot_model));
                    }

                    // Ensure, that the model error list does not grow infinitely
                    while (past_clusters.at(i).model_errors.size() > MAX_ERROR_COUNT) {
                        past_clusters.at(i).model_errors.pop_front();
                    }
                }
                ROS_INFO("Update time: %f, Size: %lu, Err cont: %f, Err model: %f, Err asp rat: %f, Err vol diff %f, Err dist cont %f, Dim: %f %f %f, Point count: %d",
                         past_clusters.at(i).last_update_time.toSec(), past_clusters.at(i).size, past_clusters.at(i).last_cont_error.overall,
                         past_clusters.at(i).model_errors.back().overall_static, past_clusters.at(i).model_errors.back().aspect_ratio, past_clusters.at(i).model_errors.back().volume_diff, past_clusters.at(i).last_cont_error.distance,
                         past_clusters.at(i).last_bb.dimensions.x, past_clusters.at(i).last_bb.dimensions.y, past_clusters.at(i).last_bb.dimensions.z, (int) past_clusters.at(i).last_bb.value);
            }
        }

        /**
         * @brief Cleanup the continuous cluster list
         * 
         * Removes all continuous clusters from the list, that 1. have not been
         * updated in the last CLUSTER_KEEP_DURATION seconds or that have not
         * been marked as valid after the MIN_DETECTION_TIME (plus one extra update
         * to ensure that there was enough time for marking them as valid)
         * 
         * @param now Current timestamp
         */
        void removeOldClusters(ros::Time now) {
            int i = 0;
            for (std::vector<ContinuousCluster>::iterator it = past_clusters.begin(); it != past_clusters.end();) {
                // Checks:
                // 1. If the cluster has been updated in the last CLUSTER_KEEP_DURATION (5.0) seconds
                // 2. If the cluster has been marked as valid after the MIN_DETECTION_TIME
                //    (plus one extra update to ensure that there was enough time for marking it as valid)
                // Otherwise, a cluster is removed
                if ((now - it->last_update_time).toSec() > CLUSTER_KEEP_DURATION || ((it->last_update_time - it->first_occurence_time).toSec() > MIN_DETECTION_TIME + (1/getUpdateRate()) && !it->valid)) {
                    past_clusters.erase(it);
                    if (i <= last_best_cluster_index) {
                        last_best_cluster_index--;
                        i--;
                    }
                } else {
                    it++;
                }
                i++;
            }
        }

        /**
         * @brief Finds the center value of three values
         * 
         * @param x
         * @param y
         * @param z
         * 
         * @return center value
         */
        double findMiddle(double x, double y, double z) {
            return std::max(std::min(x, y), std::min(std::max(x, y), z));
        }

        /**
         * @brief Calculates the error between two bounding boxes
         * 
         * @param bb Bounding box
         * @param previous_position Position of the previous cluster in a continuous cluster
         * @param previous_bb Model bounding box or bounding box of the previous cluster in a continuous cluster
         * 
         * @returns ClusterError
         */
        ClusterError calculateError(jsk_recognition_msgs::BoundingBox bb, geometry_msgs::Point::ConstPtr previous_position, jsk_recognition_msgs::BoundingBox previous_bb) {
            ClusterError error;

            // Sort values by size
            double bb_max = std::max(std::max(bb.dimensions.x, std::max(bb.dimensions.y, bb.dimensions.z)), MIN_DIMENSION_LENGTH);
            double bb_min = std::max(std::min(bb.dimensions.x, std::min(bb.dimensions.y, bb.dimensions.z)), MIN_DIMENSION_LENGTH);
            double bb_mid = std::max(findMiddle(bb.dimensions.x, bb.dimensions.y, bb.dimensions.z), MIN_DIMENSION_LENGTH);
            double mod_max = std::max(std::max(previous_bb.dimensions.x, std::max(previous_bb.dimensions.y, previous_bb.dimensions.z)), MIN_DIMENSION_LENGTH);
            double mod_min = std::max(std::min(previous_bb.dimensions.x, std::min(previous_bb.dimensions.y, previous_bb.dimensions.z)), MIN_DIMENSION_LENGTH);
            double mod_mid = std::max(findMiddle(previous_bb.dimensions.x, previous_bb.dimensions.y, previous_bb.dimensions.z), MIN_DIMENSION_LENGTH);

            // 1. Aspect ratio
            // Source: Kaku, Okada, Niijima; Similarity Measure Based on OBBTree for 3D Model Search
            // NOTE: In the paper, they use the principal axis length, which is the same as the dimension
            double cl2_cl1 = bb_min / bb_max;
            double cl3_cl1 = bb_mid / bb_max;
            double mod2_mod1 = mod_min / mod_max;
            double mod3_mod1 = mod_mid / mod_max;
            double squared_error_aspect_ratio = std::pow(cl2_cl1 - mod2_mod1, 2)
                    + std::pow(cl3_cl1 - mod3_mod1, 2);
            error.aspect_ratio = std::sqrt(squared_error_aspect_ratio);

            // 3. Volume difference
            // This is one sided, since volume_bb should never be larger than volume_model for correct correlation
            double volume_bb = bb_max * bb_mid * bb_min;
            double volume_model = mod_max * mod_mid * mod_min;
            error.volume_diff = std::abs((volume_bb / volume_model) - 1);

            // 4. Distance error
            if (previous_position != nullptr) {
                error.distance = std::sqrt(
                        std::pow(bb.pose.position.x - previous_position->x, 2) + 
                        std::pow(bb.pose.position.y - previous_position->y, 2) + 
                        std::pow(bb.pose.position.z - previous_position->z, 2)
                    );
            }

            // 4. Overall error
            // Static = without position
            error.overall_static = (WEIGHT_ASPECT_RATIO * error.aspect_ratio
                                   + WEIGHT_VOLUME_DIFF * error.volume_diff)
                                   / (WEIGHT_ASPECT_RATIO + WEIGHT_VOLUME_DIFF);   

            if (previous_position != nullptr) {
                error.overall = (WEIGHT_ASPECT_RATIO * error.aspect_ratio +
                                + WEIGHT_VOLUME_DIFF * error.volume_diff + WEIGHT_DISTANCE * error.distance)
                                / (WEIGHT_ASPECT_RATIO + WEIGHT_VOLUME_DIFF + WEIGHT_DISTANCE);   
            } else {
                // If we have no previous position, overall and overall_static are the same
                error.overall = error.overall_static;
            }

            return error;
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_obb", topic_obb) && allParametersRead;
            allParametersRead = nh.getParam("topic_no_ground_cloud", topic_no_ground_cloud) && allParametersRead;

            if (nh.hasParam("model")) {
                std::string path;
                nh.getParam("model", path);
                allParametersRead = loadModelFromFile(path) && allParametersRead;
            } else if (nh.hasParam("width") && nh.hasParam("height") && nh.hasParam("depth")) {
                nh.getParam("width", robot_model.dimensions.x);
                nh.getParam("depth", robot_model.dimensions.y);
                nh.getParam("height", robot_model.dimensions.z);
            } else {
                allParametersRead = false;
            }

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_tracking robot_detection"
                    " _topic_obb:=/cluster_obb_in_topic"
                    " _topic_no_ground_cloud:=/bg_cloud_in_topic"
                    " _model:=path_to_model_file");
            }

            return allParametersRead;
        }

        /**
         * @brief Loads robot dimensions (model) from a file
         * 
         * @param path to the file
         * 
         * @return true, when model was successfully loaded
         */
        bool loadModelFromFile(std::string path) {
            // Open file
            std::ifstream file(path);
            if (!file) {
                ROS_INFO("File %s not found", path.c_str());
                return false;
            }

            // Read file
            std::vector<float> values(3);
            for (int i = 0; i < 3; ++i) {
                if (!(file >> values[i])) {
                    ROS_INFO("Invalid file given");
                    return false;
                }
            }

            // Store values in class attributes
            robot_model.dimensions.x = values[0];
            robot_model.dimensions.y = values[1];
            robot_model.dimensions.z = values[2];

            file.close();
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_detection"); 

    RobotDetection robot_detection; 
    ROS_INFO("\033[1;32m---->\033[0m Robot detection Started.");  

    ros::spin();

    return 0;
}

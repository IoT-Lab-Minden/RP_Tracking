#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <map>

typedef pcl::PointXYZ PointType;

/**
 * @class MultiCloudMerge
 * 
 * @brief Merges point clouds from multiple sensors
 * 
 * To increase the capture area, clouds of multiple sensors can
 * be merged using this ROS node. The number of sensors and the
 * main sensor, defining the position and orientation of the
 * resulting point cloud can be specified using the paraeters. The
 * fine alignment of the point clouds is implemented in this node.
 * For rough aligment of the point clouds, it is required to provide
 * transforms using a static transform publisher. Also note, that for
 * accurate results, the timings of the individual sensors should be
 * synchronized. Furthermore, the publisher nodes of the sensors need
 * to share the same publishing frequency and every publisher node
 * requires a unique frame id.
 */
class MultiCloudMerge {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_point_cloud;

        std::string topic_in;
        std::string topic_out;

        int sensor_count; // number of sensors installed
        std::string main_frame; // frame id of the main point cloud sensor

        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
        std_msgs::Header main_header;
        std::map<std::string, Eigen::Matrix4f> transforms;

        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tf;

        bool merging = false;

    public:
        /**
         * @brief Read parameters and create subscriber, publisher, and tf listener
         */
        MultiCloudMerge() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_in, sensor_count, &MultiCloudMerge::pointCloudCallback, this);
            pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(topic_out, 1, true);

            tfBuffer = std::make_unique<tf2_ros::Buffer>();
            tf = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        }

        /**
         * @brief Receives input clouds from all sensors
         * 
         * @param point_cloud_msg
         */
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
            // Convert message to cloud data structure
            pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

            // Insert point cloud into clouds map
            clouds.insert({point_cloud_msg->header.frame_id, point_cloud});
            // Update header data, if we just received a new main cloud
            if (point_cloud_msg->header.frame_id == main_frame) {
                main_header = point_cloud_msg->header;
            }

            // If we have collected data from all sensors
            if (clouds.size() >=  sensor_count && !merging) {
                merging = true;
                mergeAndPublishCloud();
                merging = false;
            }
        }

        /**
         * @brief Merges all received clouds using their transforms and publishes the result
         */
        void mergeAndPublishCloud() {
            pcl::PointCloud<pcl::PointXYZ> sum_cloud;
            for (auto entry : clouds) {
                if (entry.first != main_frame) {
                    // Determine fine alignment, if not already available
                    if (transforms.count(entry.first) <= 0) {
                        if (!calibratePointCloud(entry.first, entry.second)) continue;
                    }
                    // Transform point cloud
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud_aligned(new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::transformPointCloud(*entry.second, *pclCloud_aligned, transforms[entry.first]);

                    // Add it to the end result
                    sum_cloud += *pclCloud_aligned;
                } else {
                    // The main sensor does not need to be transformed and
                    // we can add it directly to the end result
                    sum_cloud += *entry.second;
                }
            }

            // Convert it to a message and publish it
            sensor_msgs::PointCloud2::Ptr point_cloud_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(sum_cloud, *point_cloud_msg);
            // Reuse header from main point cloud
            point_cloud_msg->header = main_header;
            pub_point_cloud.publish(point_cloud_msg);

            ROS_INFO("Published cloud for %f, size %ld", main_header.stamp.toSec(), sum_cloud.size());
            // Clear point cloud list as they now have been published
            clouds.clear();
        }

        /**
         * @brief Aligns the current point cloud to the main point cloud
         * 
         * @param camera_id frame id
         * @param pointCloud to be aligned
         * 
         * @return true, if alignment was successful
         */
        bool calibratePointCloud(const std::string& camera_id, pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud) {
            geometry_msgs::TransformStamped transform;

            // Get rough initial alignment provided tf listener transform lookup
            try {
                transform = tfBuffer->lookupTransform(main_frame, camera_id, ros::Time());
            } catch (tf2::LookupException e) {
                ROS_WARN("Failed to get transform for %s. Ignoring point cloud", camera_id.c_str());
                return false;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

            // Downsample point clouds using VoxelGrid
            // for faster computation time
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setLeafSize (0.05f, 0.05f, 0.05f);

            voxel_grid.setInputCloud (point_cloud);
            voxel_grid.filter(*source_cloud_downsampled);

            voxel_grid.setInputCloud (clouds[main_frame]);
            voxel_grid.filter(*target_cloud_downsampled);

            // Register point clouds using GICP
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            gicp.setTransformationEpsilon(0.01);
            gicp.setMaxCorrespondenceDistance(0.2);
            gicp.setMaximumIterations(100);
            gicp.setRANSACIterations(100);  
            gicp.setInputSource(source_cloud_downsampled);
            gicp.setInputTarget(target_cloud_downsampled);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);

            Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

            // Extract the translation
            Eigen::Vector3f translation(transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z);

            // Extract the rotation
            Eigen::Quaternionf rotation(transform.transform.rotation.w,
                                        transform.transform.rotation.x,
                                        transform.transform.rotation.y,
                                        transform.transform.rotation.z);

            // Convert the rotation to a 3x3 rotation matrix
            Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();

            // Set the rotation part of the transformation matrix
            transform_matrix.block<3,3>(0,0) = rotationMatrix;

            // Set the translation part of the transformation matrix
            transform_matrix.block<3,1>(0,3) = translation;

            gicp.align(*aligned_cloud, transform_matrix);

            transform_matrix = gicp.getFinalTransformation();

            transforms.insert({camera_id, transform_matrix});

            ROS_INFO("%s has converged: %d score: %f", camera_id.c_str(), gicp.hasConverged(), gicp.getFitnessScore()); 
            return true;
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_in", topic_in) && allParametersRead;
            allParametersRead = nh.getParam("topic_out", topic_out) && allParametersRead;
            allParametersRead = nh.getParam("sensor_count", sensor_count) && allParametersRead;
            allParametersRead = nh.getParam("main_frame", main_frame) && allParametersRead;

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_preprocessing multi_cloud_merge"
                    " _topic_in:=/point_cloud_in_topic"
                    " _topic_out:=/point_cloud_out_topic"
                    " _sensor_count:=2"
                    " _main_frame:=frame_id");
            }

            return allParametersRead;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_camera_merge"); 

    MultiCloudMerge multi_cloud_merge;
    ROS_INFO("\033[1;32m---->\033[0m Multi Camera Merge Started.");  

    ros::spin();

    return 0;
}

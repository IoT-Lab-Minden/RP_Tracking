#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/PointCloud2.h>

#include <chrono>

typedef pcl::PointXYZ PointType;

/**
 * @class GroundRemoval
 * @brief ROS node for removing the ground points in point clouds
 * 
 * Ground removal is split into the two parts 1. prefiltering and 2. plane fitting.
 * In the first part, the input cloud is split into an upper and a lower part based
 * on a max_ground_height parameter. For the lower part, the ground plane is determined
 * using plane fitting. The plane fitting is only done with the first point cloud.
 * Subsequent point clouds use the same plane parameters. Based on the plane parameters,
 * points of the lower point cloud part are ignored, if they are close to the plane.
 * All other points of the lower and upper part are merged back together to form the final
 * point cloud with the ground removed.
 * 
 * It is optional to downsample the input point cloud prior to ground removal.
 */
class GroundRemoval {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_point_cloud;

        std::string topic_in;
        std::string topic_out;
        bool voxelize_cloud = false; // Activates downsampling

        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tf;

        geometry_msgs::TransformStamped transform;
        bool transform_initialized = false; // To only check for transform matrix once

        double max_ground_height = 0.0; // Expected max height of the ground plane
        double voxel_size = 0.05; // Voxel size for downsampling
        const double ground_threshold = 0.15; // Point to ground distance for a point to be classified as ground

        pcl::ModelCoefficients::Ptr coefficients;

    public:
        /**
         * @brief Initialize ground removal node, create tf listener und cloud subscriber and publisher
         */
        GroundRemoval() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            tfBuffer = std::make_unique<tf2_ros::Buffer>();
            tf = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

            sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_in, 100, &GroundRemoval::pointCloudCallback, this);
            pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(topic_out, 1);
        }

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
            pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr cloud_close_to_ground (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr cloud_above_ground (new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

            // auto time_start = std::chrono::system_clock::now();

            // 1. Downsample point cloud if desired
            if (voxelize_cloud)
                downsampleCloud(point_cloud, point_cloud);

            // 2. Transform the input cloud to the correct frame
            if (!transformCloud(point_cloud, transformed_cloud, point_cloud_msg->header.frame_id)) return;

            // 3. Split the input cloud into an upper and a lower part
            prefilterCloud(transformed_cloud, cloud_above_ground, cloud_close_to_ground);

            // 4. Only for the first cloud: Determine the ground plane
            if (coefficients == nullptr) {
                planeFitting(cloud_close_to_ground);
            }

            // 5. Filter out ground points
            filterCloud(coefficients, cloud_above_ground, cloud_close_to_ground);

            // 6. Publish the final cloud
            publishCloud(point_cloud_msg->header, cloud_above_ground);
        }
        
        /**
         * @brief Downsample the input cloud
         * 
         * @param input
         * @param output Downsampled cloud
         */
        void downsampleCloud(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& output) {
            pcl::VoxelGrid<PointType> voxelGrid;
            voxelGrid.setInputCloud(input);
            float leaf_size = voxel_size;
            voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxelGrid.filter(*output);      
        }

        /**
         * @brief Convert point cloud to the correct frame
         * 
         * @param input cloud
         * @param output transformed cloud
         * @param frame_id frame of the input cloud
         */
        bool transformCloud(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& output, std::string frame_id) {
            if (!transform_initialized) {
                try {
                    transform = tfBuffer->lookupTransform("base_link", frame_id, ros::Time());
                    // tf->~TransformListener();
                    // tfBuffer->~Buffer();
                    transform_initialized = true;
                } catch (tf2::LookupException e) {
                    ROS_WARN("Failed to get transform. Ignoring point cloud");
                    return false;
                }
            }
            pcl_ros::transformPointCloud(*input, *output, transform.transform);
            return true;
        }

        /**
         * @brief Splits the input cloud into two parts
         * 
         * @param input
         * @param above_ground points that are above the ground based on max_ground_height
         * @param close_to_ground points that are close to the ground or might be part of the ground
         */
        void prefilterCloud(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& above_ground, pcl::PointCloud<PointType>::Ptr& close_to_ground) {
            Eigen::Vector4f min_point = Eigen::Vector4f(-FLT_MAX, -FLT_MAX, -FLT_MAX, 1.0f);
            Eigen::Vector4f max_point = Eigen::Vector4f(FLT_MAX, FLT_MAX, max_ground_height, 1.0f);

            // Determine points above and below max_ground_height
            pcl::CropBox<PointType> crop_box_filter;
            crop_box_filter.setInputCloud(input);
            crop_box_filter.setMin(min_point);
            crop_box_filter.setMax(max_point);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            crop_box_filter.filter(inliers->indices);

            // Split the points into seperate point clouds
            pcl::ExtractIndices<PointType> extract_indices_filter;
            extract_indices_filter.setInputCloud(input);
            extract_indices_filter.setIndices(inliers);

            extract_indices_filter.filter(*close_to_ground);
            extract_indices_filter.setNegative(true);
            extract_indices_filter.filter(*above_ground);
        }

        /**
         * @brief Calculate ground plane parameters
         * 
         * @param close_to_ground point cloud
         */
        void planeFitting(pcl::PointCloud<PointType>::Ptr& close_to_ground) {
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<PointType> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (ground_threshold);

            seg.setInputCloud (close_to_ground);
            // Calculate plane coefficients
            seg.segment (*inliers, *coeff);

            if (coeff->values[2] < 0.9f) ROS_WARN("The detected plane might not be the ground plane. normal_z = %f", coeff->values[2]);
            ROS_INFO("Fitted plane: %f %f %f %f", coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);
            // Store coefficients in class attributes
            coefficients = coeff;
        }

        /**
         * @brief Add non-ground points of close_to_ground cloud to above_ground cloud based on plane coefficients
         * 
         * @param coefficients ground plane coefficients
         * @param above_ground cloud
         * @param close_to_ground cloud
         */
        void filterCloud(const pcl::ModelCoefficients::Ptr& coefficients, pcl::PointCloud<PointType>::Ptr& above_ground, pcl::PointCloud<PointType>::Ptr& close_to_ground) {
            for (int i = 0; i < close_to_ground->points.size(); i++) {
                if (std::abs(close_to_ground->points[i].x * coefficients->values[0]
                    + close_to_ground->points[i].y * coefficients->values[1] 
                    + close_to_ground->points[i].z * coefficients->values[2]
                    + coefficients->values[3]) > ground_threshold) {
                    above_ground->push_back(close_to_ground->points[i]);
                }
            }
        }

        /**
         * @brief Publish filtered cloud on ROS topic
         * 
         * @param header Copied header of input cloud 
         * @param cloud
         */
        void publishCloud(const std_msgs::Header& header, const pcl::PointCloud<PointType>::Ptr& cloud) {
            sensor_msgs::PointCloud2::Ptr point_cloud_msg_filtered(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud, *point_cloud_msg_filtered);
            point_cloud_msg_filtered->header = header;
            if (point_cloud_msg_filtered->header.stamp.isZero()) {
                point_cloud_msg_filtered->header.stamp = ros::Time::now();
            }
            point_cloud_msg_filtered->header.frame_id = "base_link";
            pub_point_cloud.publish(point_cloud_msg_filtered);
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_in", topic_in) && allParametersRead;
            allParametersRead = nh.getParam("topic_out", topic_out) && allParametersRead;
            if (nh.hasParam("ground_height")) {
                nh.getParam("ground_height", max_ground_height);
            }
            if (nh.hasParam("voxelize_cloud")) {
                nh.getParam("voxelize_cloud", voxelize_cloud);
                if (nh.hasParam("voxel_size")) {
                    nh.getParam("voxel_size", voxel_size);
                }
            }

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_tracking ground_removal"
                    " _topic_in:=/point_cloud_in_topic"
                    " _topic_out:=/point_cloud_out_topic"
                    " _ground_height:=0.0"
                    " _voxelize_cloud:=true"
                    " _voxel_size:=0.05");
            }

            return allParametersRead;
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_removal"); 

    GroundRemoval ground_removal; 
    ROS_INFO("\033[1;32m---->\033[0m Point Cloud Ground Removal Started.");  

    ros::spin();

    return 0;
}

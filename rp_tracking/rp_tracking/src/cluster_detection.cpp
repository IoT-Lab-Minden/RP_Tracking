#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>

#include <computeCentroidAndOBB.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Eigenvalues>
#include <eigen_conversions/eigen_msg.h>

#include "region_growing.h"

#define MAX_POINT_COUNT 2000

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB ColorType;

/**
 * @class ClusterDetection
 * @brief Finds point cluster based on a foreground and a background cloud
 * 
 * Given a foreground and background point cloud, the cluster detection finds
 * cluster based on the euclidean distance between points. Points from the
 * foreground are used to initialize individual clusters, to which points from
 * both the foreground and background clouds are added. Cluster with a high
 * foreground-to-background ratio (i.e., clusters that have a high number of
 * background points compared to foreground points) are discarded.
 */
class ClusterDetection {
    private:
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_foreground;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_complete;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
        ros::Publisher pub_point_cloud;
        ros::Publisher pub_obb;

        std::string topic_foreground;
        std::string topic_complete;

        // Treshold for valid cluster, require at least that percentage of foreground points
        float foregroundToBackgroundRatio = 0.5;
        float searchRadius = 0.2; // Neighbouring point radius

    public:
        /**
         * @brief Reads parameters and creates subscribers and publishers
         * 
         * The topics for the foreground and background clouds are synchronized.
         */
        ClusterDetection() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            sub_foreground.subscribe(nh, topic_foreground, 100);
            sub_complete.subscribe(nh, topic_complete, 100);
            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), sub_foreground, sub_complete);
            sync->registerCallback(boost::bind(&ClusterDetection::pointCloudCallback, this, _1, _2));

            pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/pc_clusters", 1);
            pub_obb = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cluster_obb", 1);
        }

        /**
         * @brief Callback for receiving the subscribed point clouds and doing cluster detection
         * 
         * This method executes the full pipeline of cluster detection. First, both input clouds
         * are downsampled to reduce computation time. Then, cluster are detected using euclidean-
         * based region growing with both input clouds. For each detected cluster, the oriented
         * bounding box is calculated.
         * 
         * @param foreground_msg ROS message containing foreground cloud data
         * @param complete_msg ROS message containing background cloud data
         */
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& foreground_msg, const sensor_msgs::PointCloud2::ConstPtr& complete_msg) {
            pcl::PointCloud<PointType>::Ptr foreground (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr complete (new pcl::PointCloud<PointType>);
            pcl::PointCloud<ColorType>::Ptr cluster_cloud (new pcl::PointCloud<ColorType>);
            pcl::fromROSMsg(*foreground_msg, *foreground);
            pcl::fromROSMsg(*complete_msg, *complete);

            // Reduce point density of both clouds
            // This is required here as high density point clouds have a very
            // high impact on computation time.
            downsampleCloud(foreground, foreground, 0.05f);
            downsampleCloud(complete, complete, 0.05f);

            std::vector<jsk_recognition_msgs::BoundingBox> bb_msgs;

            float colorValue = 0.0;

            // Initialize segmentation algorithm
            RegionGrowing<PointType> object_segmentation;
            object_segmentation.setMinClusterSize (1);
            object_segmentation.setForegroundBackgroundRatio(foregroundToBackgroundRatio);
            object_segmentation.setSearchRadius (searchRadius);
            object_segmentation.setInputCloud(complete);
            object_segmentation.setForeground (foreground);

            // Segment cluster
            std::vector<pcl::PointIndices> clusters;
            object_segmentation.extract (clusters);

            // TODO: BUG: Sometimes seems to add an unnecessary default bounding box
            // Currently unclear, why this happens. And as it does not seem to impact
            // performance, we can ignore it for now.
            for (int i = 0; i < clusters.size(); i++) {

                pcl::PointIndices::Ptr cluster(new pcl::PointIndices);
                cluster->indices = clusters[i].indices;

                // Extract cluster cloud
                pcl::ExtractIndices<PointType> extractor;
                pcl::PointCloud<PointType>::Ptr new_cluster (new pcl::PointCloud<PointType>);
                extractor.setInputCloud(complete);
                extractor.setIndices(cluster);
                extractor.setNegative(false);
                extractor.filter(*new_cluster);

                // Calculate oriented bounding box
                Eigen::Matrix<double, 3, 1> centroid;
                Eigen::Matrix<double, 3, 1> obb_center;
                Eigen::Matrix<double, 3, 1> obb_dimensions;
                Eigen::Matrix<double, 3, 3> obb_rotational_matrix;
                computeCentroidAndOBB(*new_cluster, centroid, obb_center, obb_dimensions, obb_rotational_matrix);

                // Create message from bounding box data
                jsk_recognition_msgs::BoundingBox jsk_obb;
                jsk_obb.header = foreground_msg->header;
                tf::pointEigenToMsg(obb_center, jsk_obb.pose.position);
                Eigen::Quaterniond eigen_quat(obb_rotational_matrix);
                tf::quaternionEigenToMsg(eigen_quat, jsk_obb.pose.orientation);
                tf::vectorEigenToMsg(obb_dimensions, jsk_obb.dimensions);
                jsk_obb.label = 0;
                jsk_obb.value = new_cluster->size();
                bb_msgs.push_back(jsk_obb);

                // Color cluster and add it to debug cluster cloud
                for (PointType point : new_cluster->points){
                    pcl::PointXYZHSV hsvPoint;
                    ColorType rgbPoint;
                    hsvPoint.x = point.x;
                    hsvPoint.y = point.y;
                    hsvPoint.z = point.z;

                    hsvPoint.h = (colorValue - floor(colorValue)) * 255;
                    hsvPoint.s = 0.8;
                    hsvPoint.v = 1.0;
                    pcl::PointXYZHSVtoXYZRGB(hsvPoint, rgbPoint);

                    cluster_cloud->push_back(rgbPoint);
                }

                colorValue += 0.3;
            }

            // Debug output for published the clustered (colored) point cloud
            sensor_msgs::PointCloud2::Ptr cluster_cloud_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cluster_cloud, *cluster_cloud_msg);
            cluster_cloud_msg->header = foreground_msg->header;
            pub_point_cloud.publish(cluster_cloud_msg);

            // Publishes the cluster bounding boxes
            jsk_recognition_msgs::BoundingBoxArray::Ptr bb_array_msg(new jsk_recognition_msgs::BoundingBoxArray);
            bb_array_msg->header = foreground_msg->header;
            bb_array_msg->boxes = bb_msgs;
            pub_obb.publish(bb_array_msg);
        }
        
        /**
         * @brief Reduce point density of the input cloud
         * 
         * Downsampling is required for the cluster detection, as high density
         * point clouds have a high impact on computation time. This is due to
         * the fact, that the clustering algorithm searches for all points in
         * the immediate surrounding. The algorithm has a complexity of O(n^2).
         * 
         * @param input cloud
         * @param output cloud (result)
         * @param voxel_size point density value
         */
        void downsampleCloud(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& output, float voxel_size) {
            pcl::VoxelGrid<PointType> voxelGrid;
            voxelGrid.setInputCloud(input);
            float leaf_size = voxel_size;
            voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxelGrid.filter(*output);      
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_foreground", topic_foreground) && allParametersRead;
            allParametersRead = nh.getParam("topic_complete", topic_complete) && allParametersRead;
            if (nh.hasParam("foreground_to_background_ration")) {
                nh.getParam("foreground_to_background_ration", foregroundToBackgroundRatio);
            }
            if (nh.hasParam("search_radius")) {
                nh.getParam("search_radius", searchRadius);
            }

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_tracking cluster_detection"
                    " topic_foreground:=/point_cloud_foreground"
                    " topic_complete:=/point_cloud_complete"
                    " foreground_to_background_ratio:=0.5"
                    " search_radius:=0.2");
            }

            return allParametersRead;
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster_detection"); 

    ClusterDetection cluster_detection; 
    ROS_INFO("\033[1;32m---->\033[0m Point Cloud Cluster detection Started.");  

    ros::spin();

    return 0;
}

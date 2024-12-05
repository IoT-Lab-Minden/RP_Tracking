#include <ros/ros.h>
#include <queue>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointType;

/**
 * @class ConsecutiveCloudMerge
 * 
 * @brief Merges consecutive point clouds of a single lidar sensor
 * 
 * Since many LiDAR sensors do not produce complete point clouds of a
 * scene, we need to merge multiple point clouds to produce a single
 * point cloud, the algorythm can work with. The number of source clouds
 * merged, can be controlled using the merge_count parameter. The higher
 * the value, the more information is contained in an output cloud, but
 * also the accuracy is reduced, as the output cloud contains data from
 * a larger timespan (i.e. data, that could be a few hundred milliseconds
 * old depending on the selected merge_count). To get back the higher
 * update rate, the overlap parameter can be changed. A value of 0.0 means,
 * that input cloud is only published as a part of one output cloud, which
 * drastically reduces the framerate. A value of 1.0 means, that for every
 * input cloud, a new output cloud is published. Therefore, there is no
 * reduction in framerate, but every input cloud is published multiple times
 * as a part of an output cloud.
 */
class ConsecutiveCloudMerge {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_point_cloud;

        std::string topic_in;
        std::string topic_out;
        int merge_count; // Number of input clouds to be merged for a single output cloud
        // Publish rate increase, by publishing input cloud data multiple times in
        // different output clouds
        float overlap; // 1.0 = publish every cloud, 0.0 = no overlap, 0.5 = 50% overlap

        // List of received point clouds
        std::queue<pcl::PointCloud<PointType>::Ptr> point_cloud_queue;
        int counter; // Used for keeping track of when to publish the next cloud

    public:
        /**
         * @brief Reads parameters and creates subscriber and publisher
         */
        ConsecutiveCloudMerge() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_in, 100, &ConsecutiveCloudMerge::pointCloudCallback, this);
            pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(topic_out, 1);
        }

        /**
         * @brief Callback function for the point cloud subscriber
         * 
         * @param point_cloud_msg Message
         */
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
            // Convert message to pcl type
            pcl::PointCloud<PointType>::Ptr  point_cloud_new (new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*point_cloud_msg, *point_cloud_new);

            // Add point cloud to new merge list
            point_cloud_queue.push(point_cloud_new);
            counter++;

            // Return if we don't have enough point clouds
            if (point_cloud_queue.size() < merge_count) {
                ROS_INFO("Waiting for enough point clouds, currently %i/%i", (int) point_cloud_queue.size(), merge_count);
                return;
            }

            // Only publish new data, otherwise it might be too much
            // We can divide merge_count by some value to increase framerate, but this would lead to
            // point cloud parts published twice
            ROS_INFO("Update diff %d", std::max((int)std::ceil(merge_count*(1-overlap)), 1));
            if (counter % std::max((int)std::ceil(merge_count*(1-overlap)), 1) == 0) {

                // Merge all point clouds
                pcl::PointCloud<PointType>::Ptr merged_point_cloud(new pcl::PointCloud<PointType>());
                std::queue<pcl::PointCloud<PointType>::Ptr> working_copy = point_cloud_queue;
                while (!working_copy.empty())
                {
                    auto point_cloud = working_copy.front();
                    *merged_point_cloud += *point_cloud;
                    working_copy.pop();
                }

                // Publish point cloud
                sensor_msgs::PointCloud2::Ptr merged_point_cloud_msg(new sensor_msgs::PointCloud2);
                pcl::toROSMsg(*merged_point_cloud, *merged_point_cloud_msg);
                merged_point_cloud_msg->header = point_cloud_msg->header;
                pub_point_cloud.publish(merged_point_cloud_msg);
                // ROS_INFO("Publishing point cloud with %i points", (int) merged_point_cloud->size());
            }

            // Remove first point cloud object to keep merge_count-1 point clouds
            point_cloud_queue.pop();
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_in", topic_in) && allParametersRead;
            allParametersRead = nh.getParam("topic_out", topic_out) && allParametersRead;
            allParametersRead = nh.getParam("merge_count", merge_count) && allParametersRead;
            if (nh.hasParam("overlap")) {
                nh.getParam("overlap", overlap);
            } else {
                overlap = 0.0;
            }

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_preprocessing consecutive_cloud_merge"
                    " _topic_in:=/point_cloud_in_topic"
                    " _topic_out:=/point_cloud_out_topic"
                    " _merge_count:=8"
                    " _overlap:=0.5");
            }

            return allParametersRead;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "consecutive_cloud_merge"); 

    ConsecutiveCloudMerge consecutive_cloud_merge; 
    ROS_INFO("\033[1;32m---->\033[0m Consecutive Cloud Merge Started.");  

    ros::Rate r(100); // For up to 100Hz point clouds
    while(ros::ok()){
        ros::spinOnce(); 
        r.sleep(); 
    }

    return 0;
}
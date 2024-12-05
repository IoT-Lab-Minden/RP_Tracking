#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI IntensityType;

/**
 * @class BackgroundRemoval
 * 
 * @brief Removes the static background from the background cloud
 * 
 * Creates a background cloud over time that stores the probability of points
 * being part of the background. Each point covers a certain background region
 * given by a radius. The removal algorithms removes all points from the input
 * cloud that have a corresponding point in the background cloud that has a
 * background probability above the threshold. The background cloud is updated
 * based on the input cloud. Background points that have a corresponding point
 * in the input cloud get an increase in background probability while for all
 * the other points, the background probability is decreased. This increase
 * and decrease is controlled by the parameters fadeInSpeed and fadeOutSpeed,
 * respectively.
 */
class BackgroundRemoval {
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_point_cloud;
        ros::Publisher pub_background;
        ros::Publisher pub_current_background;

        std::string topic_in;
        std::string topic_out;

        pcl::PointCloud<IntensityType>::Ptr background; // Contains the probabilities of points being in the background
        ros::Time lastRosTime;
        float fadeOutSpeed = 0.1; // it takes 5 secs for a point to leave the background
        float fadeInSpeed = 0.2; // it takes 2.5 secs for a point to join the background

        float correspondingPointDistance = 0.2f; // one background point covers a radius of 20 cm
        float backgroundTreshold = 0.5f; // points with this probability and upwards are part of the background
        float initialBackgroundProbability = backgroundTreshold + fadeOutSpeed; // points of the initial point cloud already start with this value added

    public:
        /**
         * @brief Read parameters and create subscriber and publisher
         */
        BackgroundRemoval() : nh("~") {
            if (!readParams()) {
                ros::requestShutdown();
            }

            sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_in, 100, &BackgroundRemoval::pointCloudCallback, this);
            pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(topic_out, 1);
            pub_background = nh.advertise<sensor_msgs::PointCloud2>("/background", 1);
            pub_current_background = nh.advertise<sensor_msgs::PointCloud2>("/current_background", 1);
        }

        /**
         * @brief Receive input cloud messages
         * 
         * @param point_cloud_msg Received message
         */
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
            pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr foreground (new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr current_background (new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

            ros::Time currentRosTime = point_cloud_msg->header.stamp;

            // Initialize timing and background cloud data when first point cloud was received
            bool init = false;
            if (background == NULL || lastRosTime > currentRosTime) {
                setInitialBackground(point_cloud);
                lastRosTime = point_cloud_msg->header.stamp;
                init = true;
            }

            // determine foreground and update background
            seperate(point_cloud, foreground, current_background, currentRosTime, init);
            // cleanup background (remove points with intensity zero)
            cleanupBackground();

            // Debug output for overall background cloud
            sensor_msgs::PointCloud2::Ptr background_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*background, *background_msg);
            background_msg->header = point_cloud_msg->header;
            pub_background.publish(background_msg);

            // Debug output for current background cloud
            sensor_msgs::PointCloud2::Ptr current_background_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*current_background, *current_background_msg);
            current_background_msg->header = point_cloud_msg->header;
            pub_current_background.publish(current_background_msg);

            // publish foreground point cloud
            sensor_msgs::PointCloud2::Ptr foreground_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*foreground, *foreground_msg);
            foreground_msg->header = point_cloud_msg->header;
            pub_point_cloud.publish(foreground_msg);

            lastRosTime = currentRosTime;
        }

        /**
         * @brief Searches for a corresponding background point for a given input point
         * 
         * Given a point from the input cloud, find the corresponding point in the
         * background cloud. If no point background point was found, it also looks in the
         * list of new background points from this step, that have not already been inserted
         * into the background cloud.
         * 
         * @param point from the input cloud
         * @param k_indices corresponding point in background cloud
         * @param k_distances distance to corresponding point
         * @param kd_tree KDTree of background cloud
         * @param new_background cloud of points that haven't been added to the background
         * 
         * @return 0 = none found, 1 = corresponding point found in background, 2 = found in new background
         */
        int findNearestBackgroundPoint(const pcl::PointXYZ &point, std::vector<int> &k_indices, std::vector<float> &k_distances,
                const pcl::search::KdTree<IntensityType>::Ptr &kdTree, const pcl::PointCloud<IntensityType>::Ptr &new_background) {
            // Check in current background
            IntensityType pointI;
            pointI.x = point.x;
            pointI.y = point.y;
            pointI.z = point.z;
            pointI.intensity = backgroundTreshold;
            int res = kdTree->nearestKSearch(pointI, 1, k_indices, k_distances);
            if (res == 1 && k_distances[0] <= correspondingPointDistance * correspondingPointDistance) {
                return res;
            }

            if(new_background->size() == 0) return res;

            // Check in the background points that have been added in this step
            pcl::search::KdTree<IntensityType>::Ptr kdTreeNew(new pcl::search::KdTree<IntensityType>());
            kdTreeNew->setInputCloud(new_background);
            pcl::Indices indices;
            std::vector<float> sqr_distances;
            int res2 = kdTreeNew->nearestKSearch(pointI, 1, indices, sqr_distances);
            if (res2 == 1 && sqr_distances[0] <= correspondingPointDistance * correspondingPointDistance) {
                k_indices = indices;
                k_distances = sqr_distances;
                return 2;
            }

            return res;
        }

        /**
         * @brief Seperates foreground from background
         * 
         * Every point of the input cloud is checked, if it is part of the background.
         * If it is not part of the background, it is added to the foreground cloud.
         * 
         * @param point_cloud input cloud
         * @param foreground output cloud
         * @param current_background output cloud, only used for visualization
         */
        void seperate(pcl::PointCloud<PointType>::Ptr point_cloud, pcl::PointCloud<PointType>::Ptr foreground, pcl::PointCloud<PointType>::Ptr current_background, ros::Time currentRosTime, bool init) {
            // Keeps track of newly added background points as the kdTree of the previous background points
            // does update automatically when inserting them to the actual background cloud
            pcl::PointCloud<IntensityType>::Ptr new_background (new pcl::PointCloud<IntensityType>);

            pcl::search::KdTree<IntensityType>::Ptr kdTree(new pcl::search::KdTree<IntensityType>());
            kdTree->setInputCloud(background);
            ROS_INFO("Background size %ld", background->size());

            float sqr_corr_dist = correspondingPointDistance * correspondingPointDistance;
            ros::Duration timestep = currentRosTime - lastRosTime; // Delta time for probability update
            ROS_INFO("Time: %f, Step: %f", currentRosTime.toSec(), timestep.toSec());

            std::vector<bool> update_points; // Array to store, if a background point needs to be updated
            update_points.resize(background->points.size());

            int i_add = 0;
            int i_exist = 0;
            int i_reset = 0;
            float avg_intensity = 0;
            // Check every input point, if it is in the foreground or background
            for (PointType point : point_cloud->points) {
                pcl::Indices indices;
                std::vector<float> sqr_distances;
                // Find corresponing background point in the background cloud or the new background points
                int res = findNearestBackgroundPoint(point, indices, sqr_distances, kdTree, new_background);
                if (res >= 1) {
                    // Corresponding point was found
                    if (sqr_distances[0] <= sqr_corr_dist) {
                        // Set if we need to move this point towards being background
                        if (res == 1) {
                            update_points[indices[0]] = true;
                        }

                        // If the point is part of the new background or it has a corresponding point
                        // in the current background with a low enough background probability
                        if (res == 2 || background->points[indices[0]].intensity <= backgroundTreshold) {
                            // Add point to foreground cloud
                            foreground->push_back(point);

                            i_exist++;
                        } else {
                            // Otherwise, we just add it to the current background cloud
                            current_background->push_back(point);
                        }

                        avg_intensity += background->points[indices[0]].intensity;
                        i_reset++;
                    // No corresponding point was found
                    } else {
                        // Initial intensity based on the fade in speed
                        IntensityType pointI;
                        pointI.x = point.x;
                        pointI.y = point.y;
                        pointI.z = point.z;
                        pointI.intensity = fadeInSpeed * timestep.toSec();
                        if (init) {
                            pointI.intensity = initialBackgroundProbability;
                        }
                        // Add new point for background tracking here
                        background->push_back(pointI);
                        // We need to add the point here as well because KdTree does not support point cloud updates
                        new_background->push_back(pointI);
                        // Add point to foreground as well
                        foreground->push_back(point);

                        avg_intensity += background->points[indices[0]].intensity;
                        i_add++;
                    }
                } else {
                    // Should only happen, when the background point and the new_background does not contain any point
                    ROS_WARN("ERROR: Did not find nearest point");
                }
            }

            // Update the background probability for every point in the background cloud
            for (int i = 0; i < update_points.size(); i++) {
                if (update_points[i] == true) {
                    // Increase background probability if it has been touched in this step
                    background->points[i].intensity = std::min(background->points[i].intensity + (fadeInSpeed * timestep.toSec()), 1.0);
                } else {
                    // Otherwise decrease background probability
                    background->points[i].intensity = std::max(background->points[i].intensity - (fadeOutSpeed * timestep.toSec()), 0.0);
                }
            }

            ROS_INFO("Points: %d added, %d resetted (avg intensity: %.3f), %d forwarded",
                i_add, i_reset, avg_intensity / (i_reset + i_add), i_exist);
        }

        /**
         * @brief Removes all background points with a probability value of zero or less
         */
        void cleanupBackground() {
            pcl::PointIndices::Ptr indices(new pcl::PointIndices());

            // Find all points with a probability value of zero or less
            for (int i = 0; i < background->points.size(); i++) {
                if (background->points[i].intensity <= 0.0f) {
                    indices->indices.push_back(i);
                }
            }

            ROS_INFO("Points: %lu removed", indices->indices.size());

            // And remove them from the background cloud
            pcl::ExtractIndices<IntensityType> extractor;
            extractor.setInputCloud(background);
            extractor.setIndices(indices);
            extractor.setNegative(true);
            extractor.filter(*background);
        }

        /**
         * @brief Initialize the background cloud using the first input cloud
         * 
         * Downsamples the input cloud based on the correspondingPointDistance and
         * inserts these points into the background cloud with an initial probability.
         * Each point is intialized as part of the background.
         * 
         * @param point_cloud input cloud
         */
        void setInitialBackground(pcl::PointCloud<PointType>::Ptr point_cloud) {
            pcl::PointCloud<PointType>::Ptr pc_voxelized (new pcl::PointCloud<PointType>);

            // Downsample point cloud to fit correspondingPointDistance
            pcl::VoxelGrid<PointType> voxelGrid;
            voxelGrid.setInputCloud(point_cloud);
            voxelGrid.setLeafSize(correspondingPointDistance, correspondingPointDistance, correspondingPointDistance);
            voxelGrid.filter(*pc_voxelized);

            // Set first input cloud as background cloud
            background = pcl::PointCloud<IntensityType>::Ptr(new pcl::PointCloud<IntensityType>);
            background->resize(pc_voxelized->size());
            for (int i = 0; i < pc_voxelized->size(); i++) {
                background->points[i].x = pc_voxelized->points[i].x;
                background->points[i].y = pc_voxelized->points[i].y;
                background->points[i].z = pc_voxelized->points[i].z;
                // Set initial background probability
                background->points[i].intensity = initialBackgroundProbability;
            }
        }

        /**
         * @brief Read all parameters for this node
         */
        bool readParams() {
            bool allParametersRead = true;
            allParametersRead = nh.getParam("topic_in", topic_in) && allParametersRead;
            allParametersRead = nh.getParam("topic_out", topic_out) && allParametersRead;

            if (!allParametersRead) {
                ROS_WARN(
                    "Could not read all parameters. Typical command-line usage:\n"
                    "rosrun rp_tracking background_removal"
                    " _topic_in:=/point_cloud_in_topic"
                    " _topic_out:=/point_cloud_out_topic");
            }

            return allParametersRead;
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "background_removal"); 

    BackgroundRemoval background_removal; 
    ROS_INFO("\033[1;32m---->\033[0m Point Cloud Background Removal Started.");  

    ros::spin();

    return 0;
}

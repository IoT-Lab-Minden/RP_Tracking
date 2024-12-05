#pragma once

#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <random>
#include <unordered_set>
#include <queue>
#include <pcl/common/distances.h>

/**
 * @class RegionGrowing
 * @brief Euclidean-distance based cluster detection with two inputs
 * 
 * Extracts clusters from the background and foreground clouds using the Euclidean distance
 * metric. Given a point in the foreground cloud, the algorithm extracts the corresponding
 * cluster using points from both the foreground and background cloud.
 * 
 * Adapted from: B. Wang, V. Wu, B. Wu, K. Keutzer, "LATTE: Accelerating LiDAR Point Cloud
 *               Annotation via Sensor Fusion, One-Click Annotation, and Tracking", 2019
 * Differences:
 * - No random point selection
 * - Split between foreground and background cloud
 */
template <typename PointT>
class RegionGrowing : public pcl::PCLBase<PointT>
{
public:

    using KdTree = pcl::search::Search<PointT>;
    using KdTreePtr = typename KdTree::Ptr;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    using pcl::PCLBase <PointT>::input_;
    using pcl::PCLBase <PointT>::indices_;
    using pcl::PCLBase <PointT>::use_indices_;
    using pcl::PCLBase <PointT>::initCompute;
    using pcl::PCLBase <PointT>::deinitCompute;

public:

    RegionGrowing() = default;
    ~RegionGrowing() = default;

    int getMinClusterSize() {
        return min_pts_per_cluster_;
    }
    void setMinClusterSize(int min_cluster_size) {
        min_pts_per_cluster_ = min_cluster_size;
    }

    double getForegroundBackgroundRatio() {
        return foreground_background_ratio_;
    }
    void setForegroundBackgroundRatio(double foreground_background_ratio) {
        foreground_background_ratio_ = foreground_background_ratio;
    }

    int getMaxClusterCount() {
        return max_cluster_count_;
    }
    void setMaxClusterCount(int max_cluster_count) {
        max_cluster_count_ = max_cluster_count;
    }

    double getMaxClusterRadius() {
        return max_cluster_radius_;
    }
    void setMaxClusterRadius(double max_cluster_radius) {
        max_cluster_radius_ = max_cluster_radius;
    }

    double getSearchRadius() {
        return search_radius_;
    }
    void setSearchRadius(double search_radius) {
        search_radius_ = search_radius;
    }

    void setForeground(PointCloudPtr foreground) {
        foreground_ = foreground;
    }

    virtual void extract(std::vector<pcl::PointIndices>& clusters) {

        // Clear cluster vector
        clusters.clear();

        // initialize filter
        bool init_success = initCompute();

        if (input_->size() <= 0) {
            ROS_INFO("RegionGrowing: Received empty input cloud");
            deinitCompute();
            return;
        }

        // Precalculate square of max_cluster_radius
        float sqr_max_cluster_radius = max_cluster_radius_ * max_cluster_radius_;

        // Variables for kdTree results
        pcl::Indices neighbours;
        std::vector<float> distances;
        // Init KDTree
        search_.reset(new pcl::search::KdTree<PointT>);
        search_->setInputCloud(input_);

        std::unordered_set<int> foreground_indices;

        // Check if indices are already given
        if (use_indices_) {
            // If so, store them in the set
            foreground_indices = std::unordered_set<int>(indices_->begin(), indices_->end());
        } else if (foreground_ != nullptr && !foreground_->empty()) {
            // Otherwise use foreground cloud and store its point indices in the
            // indice set
            for(PointT foreground_point : *foreground_) {
                // Find corresponding point in background cloud
                search_->nearestKSearch(foreground_point, 1, neighbours, distances);
                foreground_indices.insert(neighbours[0]);
            }
        } else {
            ROS_WARN("No foreground provided for clustering. Stopping");
            deinitCompute();
            return;
        }

        int cluster_count = 0;
        // Find clusters as long as there are points in the foreground
        while (!foreground_indices.empty() && clusters.size() < max_cluster_count_) {
            // Start with the first foreground point
            int initial_index = *foreground_indices.begin();

            // Initialize variables
            pcl::Indices cluster_indices;
            std::unordered_set<int> cluster;
            std::queue<int> index_queue;
            index_queue.push(initial_index);
            cluster.insert(initial_index);
            int num_foreground = 1, num_overall = 1;
            foreground_indices.erase(initial_index);

            // While we have points, we have not searched neighbours for
            while (!index_queue.empty()) {
                // Get the next point in queue
                int current_index = index_queue.front();
                index_queue.pop();

                // And search for all of its neighbours based on a search radius
                search_->radiusSearch(input_->at(current_index), search_radius_, neighbours, distances);
                for (int n : neighbours) {
                    // If we have not reached the maximum cluster radius
                    if (pcl::squaredEuclideanDistance(input_->at(initial_index), input_->at(n)) < sqr_max_cluster_radius) {
                        // Add it to the cluster if it is not already
                        if (cluster.insert(n).second) {
                            // And if so also add it to the index queue (To add it to the cluster and search for more neighbours)
                            index_queue.push(n);

                            // And erase point from the foreground if it was part of the foreground
                            int rm_count = foreground_indices.erase(n);

                            // Store data to calculate the foreground-background-ratio
                            if (rm_count > 0) {
                                num_foreground++;
                            }
                            num_overall++;
                        }
                    }
                }

                // Speedup computation time by breaking early
                // Note: This could lead to errors at very specific edge cases, i.e. objects
                // that are partially in the background and that have a specific form
                if ((((double) num_foreground) / num_overall) < foreground_background_ratio_) break;
            }

            double fg_bg_ratio = ((double) num_foreground) / num_overall;
            // Add this as a valid cluster if we reached the minimum number of points
            // and if the foreground-background-ratio is higher than the treshold
            if (cluster.size() >= min_pts_per_cluster_ && fg_bg_ratio >= foreground_background_ratio_) {
                // Convert set to pcl::Indices
                cluster_indices.resize(cluster.size());
                std::copy(cluster.begin(), cluster.end(), cluster_indices.begin());
                // And add it to the cluster list
                pcl::PointIndices point_indices;
                point_indices.header = input_->header;
                point_indices.indices = cluster_indices;
                clusters.push_back(point_indices);

                // Print cluster info
                ROS_INFO("Extracted valid cluster %d, %lu points, %lu points left, ratio: %f (%d/%d)",
                        cluster_count, cluster.size(), foreground_indices.size(), fg_bg_ratio, num_foreground, num_overall);

                cluster_count++;
            }
        }

        // Deinit because we have found all clusters
        deinitCompute();
    }

protected:

    // Minimum number of points in a cluster to be returned and counted
    // as a valid cluster.
    int min_pts_per_cluster_{1};

    // Stops after the given number of clusters has been extracted
    int max_cluster_count_{std::numeric_limits<int>::max()};

    // Ratio between the number of foreground and background points in a cluster
    // If the ratio exceeds this value, the cluster is considered a foreground cluster
    // and is added to the detected clusters
    double foreground_background_ratio_{0.2};

    // Maximum radius of a cluster. Points outside this radius are ignored even if
    // they have neighbouring points that are part of the cluster.
    double max_cluster_radius_{std::numeric_limits<double>::max()};

    // Neighbouring point inside this radius of another point, that is part of a
    // cluster, are also considered to be part of the cluster.
    double search_radius_{0.1};

    PointCloudPtr foreground_;

    KdTreePtr search_{nullptr};

};

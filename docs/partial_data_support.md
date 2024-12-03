# Partial data support

Many types of LiDAR sensors only provide partial data from a single capture that does not show the full capture area. To still be able to use this data, we need to merge it first.

### 1. Sensor setup

To use the data provided by a sensor, it needs to be published using a ROS topic of the type *sensor_msgs/PointCloud2*. Due to the different types of sensors, we do not provide an implementation for this. But for many sensor types, an implementation can be found online.

### 2. consecutive_cloud_merge node

The *consecutive_cloud_merge* node can be executed using the following command:

```bash
rosrun rp_preprocessing consecutive_cloud_merge
    _topic_in:=/point_cloud_in_topic
    _topic_out:=/point_cloud_out_topic
    _merge_count:=8
    _overlap:=0.5
```

Depending on your topic names, their parameters need to updated to the values, you are using. The parameter *merge_count* defines the number of input clouds, that are merged into a single output cloud. Note that with higher values, the output cloud represents a larger timespan, which decreases the accuracy. To change the publishing frequency, the *overlap* parameter can be changed to values between 0.0 and 1.0. A value of 0.0 means, that input data is only published as a part of a single output cloud. This can drastically reduce the framerate. A value of 1.0 means, that for every input cloud, a new output cloud is published. Therefore, there is no reduction in framerate, but input data is reused in multiple output clouds.

### Combination with multi sensor support

If both partial data support and multi sensor support are used, the data of a single sensor needs to be combined first using the *consecutive_cloud_merge*. Afterwards, *multi_cloud_merge* can be applied.

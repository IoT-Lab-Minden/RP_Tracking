# Multi camera support

Multiple sensors are supported through the *multi_cloud_merge* ROS node. Depending on the hardware ressources of the computer, it is possible to merge an unlimited amount of sensors.

### 1. Single sensor setup

To combine multiple sensors, it is required, that the sensors are synchronized and therefore share the same update frequency. Every sensor needs to publish its data on a shared ROS topic of the type *sensor_msgs/PointCloud2*. Due to the different types of sensors, we do not provide an implementation for this. But for many sensor types, an implementation can be found online. Please make sure that every sensor uses a different frame id.

### 2. Sensor alignment

To align multiple sensors, a rought alignment needs to be provided through the */tf* topic. The following listing shows the command:

```bash
rosrun tf static_transform_publisher X Y Z QX QY QZ QW MAIN_FRAME SENSOR_FRAME 100
```

Please replace the values written in all caps with the transformation data as well as frame names. Each sensor, except the base sensor, is aligned with the frame of the base sensor. The fine alignment is done automatically using point cloud registration.

### 3. multi_cloud_merge node

The *multi_cloud_merge* node can be executed using the following command:

```bash
rosrun rp_preprocessing multi_cloud_merge
    _topic_in:=/point_cloud_in_topic
    _topic_out:=/point_cloud_out_topic
    _sensor_count:=2
    _main_frame:=frame_id
```

Depending on your topic names, their parameters need to updated to the values, you are using. The *sensor_count* parameter specifies the number of sensors, that are publishing data on the input topic. The *main_frame* is the frame id of the base sensor. All point clouds are aligned to this frame id.

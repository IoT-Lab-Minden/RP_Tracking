# Demo

We provide a demo *rosbag* file, so you can test the proposed software yourself. First, you need to install the software as described in the [installation documentation](installation.md). The demo file can be downloaded from [this link](https://hsbi.sciebo.de/s/e6GRKP8a9IFpgb3). Next, the RP Tracking software needs to be executed using the following command.

```bash
roslaunch rp_tracking rp_tracking.launch model:=CATKIN_WS/src/rp_tracking/rp_tracking/rp_tracking/models/tello_cage.model input_cloud:=/a_monstar ground_height:=-0.7
```

Please replace *CATKIN_WS* with the path to your workspace. When directly cloning the repository into the *src* folder of your workspace, the remaining part of the path is correct. Don't forget to *source* the catkin workspace prior to starting the launch file. The demo data has been captured using two pico Monstar ToF sensor with an update frequency of 5 hz. Since the data has already been merged, the *multi_cloud_merge* node is not required. 

You can now start playing the *rosbag* file using the following command. Note, that it takes some time for the tracking to start, as this algorithm requires some initialization time. Furthermore, the demo includes an example, where the sensors are unable to capture the drone, but the algorithm is able to recover from this error.

```bash
rosbag play -l tello_cage01_2024-08-01-09-34-28.bag
```

If you want to get access to the evaluation data, please write to the e-mail mentioned in the paper.

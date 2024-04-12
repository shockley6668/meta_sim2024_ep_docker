This repository is the MetaSim team's solution for the Icra Simtoreal ([http://www.sim2real.net/track/track?nav=RMUS2024&type=nav&t=1712890299058](http://www.sim2real.net/track/track?nav=RMUS2024&type=nav&t=1712890299058)) competition.

For detailed technical reports, please refer to the release notes.

The main code for this project is located in the src directory and includes the following packages:

* **bt_frame:** Behavior tree logic code
* **carto_navigation:** Navigation package based on cartographer and movebase
* **rmus_solution:** Template matching recognition
* **simple_planner:** Path planning and trajectory tracking based on cubic spline interpolation
* **apriltag_ros:** Modified and custom-generated apriltag package, mainly used for number recognition
* **apriltag:**

## Update History:

**First-time use:**

1. Build the project: Go to the `/ICRA2024-Sim2Real-RM` directory and run `CLIENT_IMAGE=meta_sim/test:v1.3 bash scripts/build.sh`

**Navigation Methods:**

There are currently two navigation methods:

1. **movebase:** This method uses the movebase: [移除了无效网址] package for navigation.
2. **simple planner:** This method uses the `simple_planner` package for navigation.

**Using the simple planner:**

1. Modify the `planner.launch` file in `src/simple_planner/launch`:

```xml
<param name="point_num" value="3"/>
```

This means that three points are interpolated using cubic spline interpolation to create a smooth curve. You can make the curve smoother by modifying the `value`, but you will need to add more points.

```xml
/target_yaw` The yaw angle (rad) of the target point
/clicked_point` Used to publish discrete points, message type: geometry_msgs/PointStamped
```

**Changes made on 2024.2.14:**

1. Modified the duplicate removal logic in `apriltag_ros`. Now, when `remove_duplicates` is not set to `false` in `setting.yaml`, multiple tags with the same ID will be automatically removed, **and only one detection result for this ID will be retained**. The filtering criterion is: when multiple tags with the same ID are detected, the tag with the **largest area** is selected as the result.

2. Changed the image enhancement method in `apriltag_ros` from *binarization* to *contrast enhancement* in the hope of achieving better robustness in real-world testing.

3. (For debugging image processing later) Added a `threshold_image` package that can read in topics for image processing and then publish binarized or contrast-enhanced image topics. This package has two nodes:

```bash
contrast_node // Contrast image
threshold_node // Binaries image
```

To use it, enter

```bash
rosrun threshold threshold_node
or
rosrun threshold contrast_node
```

If you cannot find the `threshold` package, please go to `working directory/devel/lib/threshold` and run the executable directly

**Using images in apriltag_ros:**

```bash
roslaunch apriltag_ros continuous_detection.launch image_rect:=/threshold/image camera_info_topic:=/threshold/camera_info
or
roslaunch apriltag_ros continuous_detection.launch image_rect:=/contrast/image camera_info_topic:=/contrast/camera_info
```

4. Modified the Dockerfile to install rqt and the package for keyboard control of the robot

```bash
apt-get install -y ros-noetic-teleop-twist-keyboard ros-noetic-rqt ros-noetic-rqt-common-plugins
```

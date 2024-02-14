# meta_sim2024_ep_docker
 meta_sim2024_ep_docker

第一次用需要先build一下

2024.2.14  oplin 更改记录
1. 将 `apriltag_ros` 中的去除重复逻辑进行了修改，
现在当 `setting.yaml` 中不设置 `remove_duplicates`为false时，会自动去除检测到的多个同一id的tag，**并保留唯一一个此id的tag检测结果**。
筛选的标准为： 当检测到多个同一id的tag时，会筛选出其中**面积最大**的那个tag作为结果。
2. 将 `apriltag_ros` 中对图像增强的方式由*二值化*更改成了*提高对比度*，希望能实现在实地测试中更好的鲁棒性。
3. (用于后面 画面处理的调试)加入了一个 `threshold_image` 的包，可以读入话题进行图像处理，然后发布二值化或者对比度增强后的图像话题。
这个包有两个节点, 分别是

```shell
contrast_node // 对比度画面
threshold_node // 二值化画面
```

如果要使用, 请输入
```shell
rosrun threshold threshold_node
or
rosrun threshold contrast_node
```
如果找不到threshold这个包, 请前往 `工作目录/devel/lib/threshold` 下直接运行可执行文件
** 在 apriltag_ros中使用画面: **
```shell
roslaunch apriltag_ros continuous_detection.launch image_rect:=/threshold/image camera_info_topic:=/threshold/camera_info
or
roslaunch apriltag_ros continuous_detection.launch image_rect:=/contrast/image camera_info_topic:=/contrast/camera_info
```
4. 修改了Dockerfile，安装了rqt还有用于键盘操控机器人的包
```shell
apt-get install -y ros-noetic-teleop-twist-keyboard ros-noetic-rqt ros-noetic-rqt-common-plugins
```

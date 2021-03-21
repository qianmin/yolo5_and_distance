# detection_and_distance
yolov5、激光雷达融合感知
# 如何用yolov5发送检测的边界框
## 启动！！
 - 启动激光雷达
   ```Shell
   cd projects/leishen_ws/
   source devel/setup.bash
   roslaunch lslidar_c32_decoder lslidar_c32.launch 
   ```
 - 启动激光雷达转换
   ```Shell
   roslaunch rslidar_to_velodyne rslidar_to_vdyne.launch 
   ```
 - 启动原始图像
   ```Shell
   roslaunch usb_cam usb_cam-test.launch
   ```
 - 启动矫正图像
   ```Shell
   ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
   ```
 - 启动lidar_camera_fusion
   ```Shell
   rosrun yolo_rect yolo_rect_pub.py
   roslaunch lidar_camera_fusion yolo_distance.launch 
   rosrun lidar_camera_fusion yolo_dtance_show.py 
   
   roslaunch lidar_camera_fusion lidar_camera_fusion.launch
   ```
 - 融合图像话题
   `/image_fusion`

## 附图
   ```Shell
                               \     /    Initial rotation:
                                \ |z/     [0 -1  0]
                                 \|/      [0  0 -1]
                                  █————x  [1  0  0]
                               forward    => (pi/2, -pi/2, 0) Euler angles
                                 cam_1    Final rotation = Average rotation * Initial rotation
  
                                █████
                   |x         ██  |x ██
   [1  0  0]       |         █    |    █                 [-1 0  0]
   [0  0 -1]  z————█ cam_4  █ y———.z    █  cam_2 █————z  [0  0 -1]
   [0  1  0]                 █         █         |       [0 -1  0]
   => (pi/2, 0, 0)            ██     ██          |x      => (-pi/2, 0, pi)
                                █████
                                lidar

                             x————█       [0  1  0]
                                  |       [0  0 -1]
                                  |z      [-1 0  0]
                                 cam_3    => (pi/2, pi/2, 0)
   ```

# 2yolo
```
yolov5 index 和id不是一个东西

cls =index
names='tv'
```

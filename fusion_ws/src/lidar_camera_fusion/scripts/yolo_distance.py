#!/usr/bin/env python
# -*- coding: UTF-8 -*- 

import cv2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from yolo_rect.msg import Boxes, One_box
import tf
import sensor_msgs
import rospy
import sys
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
import struct
import math
import numpy as np
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import message_filters
import time

from numpy_pc2 import pointcloud2_to_xyz_array
from calib import Calib
from jet_color import Jet_Color

class LidarImage:
    def __init__(self):
        # print('++++++++++++++++++++++++++++++++++++++++++++++++++++')#只执行一次这句话
        self._imageInput = message_filters.Subscriber(image_topic_name, Image)
        self._velodyne = message_filters.Subscriber(lidar_topic_name, PointCloud2)
        self._pub_img_depth = rospy.Publisher("/image_fusion", Image, queue_size=1)
        
        self._ts = message_filters.ApproximateTimeSynchronizer([self._velodyne, self._imageInput], 1, 0.05)
        self._ts.registerCallback(self.lidar_camera_fusion)

        #my_code
        self.boxes_topicName = "/yolo_rect"
        rospy.Subscriber(self.boxes_topicName, Boxes, self.yolo_distance)


        self.out_boxes_name="/yolo_distance"
        self.distance_pub=rospy.Publisher(self.out_boxes_name, Boxes, queue_size=100)

        self.final_uv=0
        self.final_xyz=0
        #my_code

    def lidar_camera_fusion(self, point_cloud_data, image_data):
        # print('++++++++++++++++++++++++++++++++++++++++++++++++++++')#循环执行
        time_start_all = time.time()
        try:
            self.cvImage = CvBridge().imgmsg_to_cv2(image_data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        image_depth = []
        image_depth = self.cvImage.copy()
        height, width = image_depth.shape[:2]
        
        time_start_transform = time.time()
        pointXYZ_raw = pointcloud2_to_xyz_array(point_cloud_data, remove_nans=True)
        pointXYZ = pointXYZ_raw.copy()
        alpha = 90 - 0.5 * the_field_of_view
        k = math.tan(alpha * math.pi / 180.0)
        if the_view_number == 1:
            pointXYZ = pointXYZ[np.logical_and((pointXYZ[:, 0] > k * pointXYZ[:, 1]), (pointXYZ[:, 0] > -k * pointXYZ[:, 1]))]
        elif the_view_number == 2:
            pointXYZ = pointXYZ[np.logical_and((-pointXYZ[:, 1] > k * pointXYZ[:, 0]), (-pointXYZ[:, 1] > -k * pointXYZ[:, 0]))]
        elif the_view_number == 3:
            pointXYZ = pointXYZ[np.logical_and((-pointXYZ[:, 0] > k * pointXYZ[:, 1]), (-pointXYZ[:, 0] > -k * pointXYZ[:, 1]))]
        elif the_view_number == 4:
            pointXYZ = pointXYZ[np.logical_and((pointXYZ[:, 1] > k * pointXYZ[:, 0]), (pointXYZ[:, 1] > -k * pointXYZ[:, 0]))]
        
        pointXYZ = pointXYZ[np.logical_and((pointXYZ[:, 0] ** 2 + pointXYZ[:, 1] ** 2 > the_min_distance ** 2), (pointXYZ[:, 0] ** 2 + pointXYZ[:, 1] ** 2 < the_max_distance ** 2))]
        pointXYZ = pointXYZ[np.logical_and((pointXYZ[:, 2] > the_view_lower_limit - the_sensor_height), (pointXYZ[:, 2] < the_view_higher_limit - the_sensor_height))]
        
        cloud_xyz = calib.lidar_to_cam.dot(pointXYZ.T).T
        cloud_uv = calib.lidar_to_img.dot(pointXYZ.T).T
        cloud_uv = np.true_divide(cloud_uv[:, :2], cloud_uv[:, [-1]])
        camera_xyz = cloud_xyz[(cloud_uv[:, 0] >= 0) & (cloud_uv[:, 0] < width) & (cloud_uv[:, 1] >= 0) & (cloud_uv[:, 1] < height)]
        camera_uv = cloud_uv[(cloud_uv[:, 0] >= 0) & (cloud_uv[:, 0] < width) & (cloud_uv[:, 1] >= 0) & (cloud_uv[:, 1] < height)]
        transform_time_cost = time.time() - time_start_transform

        self.final_uv=camera_uv
        self.final_xyz=camera_xyz
        time_start_display = time.time()
        jc = Jet_Color()
        depth = np.sqrt(np.square(camera_xyz[:, 0]) + np.square(camera_xyz[:, 1]) + np.square(camera_xyz[:, 2]))
        for pt in range(0, camera_uv.shape[0]):
            cv_color = jc.get_jet_color(depth[pt] * jet_color)
            cv2.circle(image_depth, (int(camera_uv[pt][0]), int(camera_uv[pt][1])), 1, cv_color, thickness=-1)
        display_time_cost = time.time() - time_start_display
        
        try:
            self._pub_img_depth.publish(CvBridge().cv2_to_imgmsg(image_depth, 'bgr8'))
        except CvBridgeError as e:
            print(e)
        
        total_time_cost = time.time() - time_start_all
        # print(pointXYZ_raw.shape, camera_xyz.shape, camera_uv.shape)
        # print("transform time cost:", transform_time_cost)
        # print("display time cost:", display_time_cost)
        # print("total time cost: ", total_time_cost)
        # print()
    def yolo_distance(self,data):
        distance_yolo=Boxes()
        print('yolo_distance++++++++++++++++++++++++++++++++++++++++++++++++++++')#循环执行
        # print('final_uv',self.final_uv)
        uv=self.final_uv
        xyz=self.final_xyz
        if(data is not None):
            for box in data.boxes:
                box.distance=self.get_box_distance(uv,xyz,box.x1,box.x2,box.y1,box.y2)
        self.distance_pub.publish(data)

    def get_box_distance(self,uv,xyz,x1,x2,y1,y2):
        test=np.where((uv[:,0]>x1) & (uv[:,0]<x2) & (uv[:,1]>y1) & (uv[:,1]<y2))
        get_uv=uv[test]
        get_xyz=xyz[test]
        depth = np.sqrt(np.square(get_xyz[:, 0]) + np.square(get_xyz[:, 1]) + np.square(get_xyz[:, 2]))
        print(depth.min())
        return depth.min()


if __name__ == '__main__':
    try:
        rospy.init_node("fusion")
        image_topic_name = rospy.get_param("~image_topic")
        lidar_topic_name = rospy.get_param("~lidar_topic")
        
        calib = Calib()
        file_path = rospy.get_param("~calibration_file_path")
        
        the_view_number = rospy.get_param("~the_view_number")
        the_field_of_view = rospy.get_param("~the_field_of_view")
        
        the_sensor_height = rospy.get_param("~the_sensor_height")
        the_view_higher_limit = rospy.get_param("~the_view_higher_limit")
        the_view_lower_limit = rospy.get_param("~the_view_lower_limit")
        the_min_distance = rospy.get_param("~the_min_distance")
        the_max_distance = rospy.get_param("~the_max_distance")
        
        jet_color = rospy.get_param("~jet_color")
        
        calib.loadcalib(file_path)
        LidarImage()
        # print('++++++++++++++++++++++++++++++++++++++++++++++++++++')#只执行一次这句话
        



        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: UTF-8 -*- 

import cv2
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
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
from utils.Color import Color
import message_filters   #to synchronize topic subscription
import time


# ALPHA = -2.4 * math.pi / 180. 
# BETA = 3.5 * math.pi / 180.
# GAMMA = 0 * math.pi / 180.

ALPHA = -1.1 * math.pi / 180.  
BETA = -0.2 * math.pi / 180.
GAMMA = 0 * math.pi / 180.

m = 10 * math.pi / 180.
n = 10 * math.pi / 180.
k = 10 * math.pi / 180.


# """
# 相机内参
# """
# T_velo_to_cam = np.array([[920.6949462890625,                 0,    348.0517667538334,              0],
# 							[       0.000000, 934.1571044921875,    177.8464786820405,              0],
# 							[       0.000000,          0.000000,             1.000000,              0],
# 							[       0.000000,          0.000000,                    0,              1]])
# print("     ----------- T_velo_to_cam -----------")#内参
# print(T_velo_to_cam)
# print("\n\n")


# """
# 外参矩阵
# """
# P_velo_to_cam = np.array([[-0.0601795, -0.998042, -0.0170758, 0.129992],
#                     [-0.0317277, 0.0190107, -0.999316, 0.0185626],
#                     [0.997683, -0.0595965, -0.0328096, 0.0337639],
#                     [0,            0,              0,                1]])

# print("----------- P_velo_to_cam -----------")#外参
# print(P_velo_to_cam)
# print("\n\n")


# """
# 校正矩阵
# """
# H_2 = np.array([[0.974413,      0.13993,     0.175897, -0.000815583],
# 				[-0.170656,     0.969878,     0.173822, -0.000383258],
# 				[ -0.146276,    -0.199392,     0.968941, -0.000405252],
# 				[0,          0,          0,          1]])
# print("----------- H2矩阵 -----------")
# print(H_2)
# print("\n\n")


"""
相机内参
"""
#T_velo_to_cam = np.array([[593.774597,                        0,           620.058332,              0],
#							[       0.000000,        608.614380,           338.151874,              0],
#							[       0.000000,          0.000000,             1.000000,              0],
#							[       0.000000,          0.000000,                    0,              1]])
#							
T_velo_to_cam = np.array([1162.155518 0.000000 597.084363 0.000000
0.000000 1302.059326 380.907441 0.000000
0.000000 0.000000 1.000000 0.000000)
print("     ----------- T_velo_to_cam -----------")#内参
print(T_velo_to_cam)
print("\n\n")


"""
外参矩阵
"""
P_velo_to_cam = np.array([[-0.559014,-0.216072,2.46186,-1.04059],
                    [2.08989,-1.46804,300,0.0540892,-0.233175],
                    [2.32362,-1.29406,2.16713,-1.52173],
                    [0,            0,              0,                1]])
print("----------- P_velo_to_cam -----------")#外参
print(P_velo_to_cam)
print("\n\n")


"""
校正矩阵
"""
# H_2 = np.array([[ 0.969787,  0.171048, -0.173941,  0.00592294],
# 				[-0.141304,  0.975079,  0.171037,  2.09762e-06],
# 				[ 0.198862, -0.141291,  0.969789,  0.00175334],
# 				[0,          0,          0,          1]])

H_2 = np.array([[ 0.969246,  0.171956, -0.176049,  0.0119376],
				[-0.141987,  0.975046,  0.170662,  0.00220948],
				[ 0.201002, -0.140417,  0.969475,  -0.00166232],
				[0,          0,          0,          1]])
print("----------- H2矩阵 -----------")
print(H_2)
print("\n\n")

"""
调整矩阵
"""
rotx = np.array([[              1,               0,                0,           0],
                [               0, math.cos(ALPHA), -math.sin(ALPHA),           0],
                [               0, math.sin(ALPHA),  math.cos(ALPHA),           0.],
                [               0,               0,                0,           1]])

roty = np.array([[math.cos(BETA),              0,   math.sin(BETA),           0],
                [              0,              1.,               0,           0],
                [-math.sin(BETA),              0,   math.cos(BETA),           0.],
                [              0,              0,                0,           1]])

rotz = np.array([[math.cos(GAMMA),   -math.sin(GAMMA),           0,         0],
                [ math.sin(GAMMA),    math.cos(GAMMA),           0,         0],
                [               0,                  0,           1.,        0.],
                [               0,                  0,           0,         1]])

H_3 = rotz.dot(roty.dot(rotx))
print("----------- H3矩阵 -----------")
print(H_3)
print("\n\n")


"""
偏置矩阵
"""
Mx = np.array([[              1,               0,                0,                   0],
                [               0, math.cos(m), -math.sin(m),                    0],
                [               0, math.sin(m),  math.cos(m),                   0],
                [               0,               0,                0,                  1]])

My = np.array([[math.cos(n),              0,   math.sin(n),           0],
                [              0,              1.,               0,           0],
                [-math.sin(n),              0,   math.cos(n),           0.],
                [              0,              0,                0,           1]])

Mz = np.array([[math.cos(k),   -math.sin(k),           0,         0],
                [ math.sin(k),    math.cos(k),           0,         0],
                [               0,                  0,           1.,        0],
                [               0,                  0,           0,         1]])

NOISEMat = Mz.dot(My.dot(Mx))
print("----------- NOISEMat矩阵 -----------")
print(NOISEMat)
print("\n\n")





"""
转换矩阵：（内参×外参）
"""
P_velo_to_cam = np.dot(H_3,P_velo_to_cam)
# P_velo_to_cam = np.dot(NOISEMat, P_velo_to_cam)
# P_velo_to_cam = np.dot(H_2, np.dot(NOISEMat, P_velo_to_cam))

P_velo_to_img = np.dot(T_velo_to_cam,P_velo_to_cam)

print("     ----------- P_velo_to_img -----------")#外参
print(P_velo_to_img)
print("\n\n")

"""
融合函数
"""

class LidarImage: #类名通常是大写开头的单词，

	def __init__( self ):
		self._velodyne = message_filters.Subscriber( '/pandar_points2', PointCloud2 )
		self._imageInput = message_filters.Subscriber( '/usb_cam/image_rect_color2', Image )
		
		self._pub_img_depth = rospy.Publisher("/img_depth", Image, queue_size=1)
		self.pub_points_rgb = rospy.Publisher("/pandar/points_rgb", PointCloud2, queue_size=1000000)
		
		self._ts = message_filters.ApproximateTimeSynchronizer([ self._velodyne, self._imageInput], 1, 10)
		self._ts.registerCallback(self.lidar_camera_fusion)
		
	def lidar_camera_fusion(self, point_cloud_data, image_data):

		try:
			self.image = CvBridge().imgmsg_to_cv2( image_data, 'bgr8' ) #这里cvImage就是读取的图像
		except CvBridgeError as e:
			print( e )

		image_depth = []
		image_depth = self.image.copy()
		height, width = image_depth.shape[:2]

		cloud_points = []
		for p in pc2.read_points(point_cloud_data, skip_nans=True): #从ROS消息中读点云数据，5 data (2.138409376144409, 0.7059696912765503, -0.35667064785957336, 71.0, 3)
			cloud_points.append([ p[ 0 ], p[ 1 ], p[ 2 ],  1])  #把读取的点云消息存入cloud_points 
		point_cloud = np.array(cloud_points)    #类型转换为数组


		# pointXYZ = point_cloud.copy()[np.logical_and((point_cloud.copy()[ :, 1 ] > 1.1*point_cloud.copy()[ :, 0 ]) , (-point_cloud.copy()[ :, 1 ] > 1.1*point_cloud.copy()[ :, 0 ]))]
		# pointXYZ = point_cloud.copy()[(point_cloud.copy()[ :, 2 ] < 5) & (point_cloud.copy()[ :, 0 ] > 12) & (point_cloud.copy()[ :, 0 ] < 21) & (point_cloud.copy()[ :, 1 ] > -20) & (point_cloud.copy()[ :, 1 ] < 20)]
		pointXYZ = point_cloud.copy()[(point_cloud.copy()[ :, 0 ] > 0) & (point_cloud.copy()[ :, 0 ] < 10) & (point_cloud.copy()[ :, 1 ] > -5) & (point_cloud.copy()[ :, 1 ] <5)]

		# pointXYZ = point_cloud.copy()[np.where(point_cloud.copy()[:, 0] > 0)]

		pc_rgb = np.zeros((pointXYZ.shape[0], 7), dtype=np.float32) 
		pc_rgb[:, :3] = pointXYZ[:, :3]

		transformedPoint = P_velo_to_cam.dot(pointXYZ.T).T
		depth = np.sqrt(np.square(transformedPoint[:, 0]) + np.square(transformedPoint[:, 1]) + np.square(transformedPoint[:, 2]))
		depth_min = min(depth)  #1.7518548957573246
		depth_max = max(depth)  #79.729618041866885
		depth = (depth - depth_min) / (depth_max - depth_min)


		uv = P_velo_to_img.dot(pointXYZ.T).T

		uv[:,0] = np.divide(uv[:,0], uv[:, 2])
		uv[:,1] = np.divide(uv[:,1], uv[:, 2])
		uv[:,:2] = np.round(uv[:,:2]).astype(np.uint16)

		halflenth = 1
		for pt in range(0, uv.shape[0]):
		# add point only if it's within the image bounds
			if (uv[ pt ][ 0 ] >= 0 and uv[ pt][ 0 ] < width) and (uv[ pt][ 1 ] >= 0 and uv[ pt][ 1 ] < height):
			
#				color =   (self.image[ uv[ pt][ 1 ] ][ uv[ pt ][ 0 ] ][2] << 16) | (self.image[ uv[ pt][ 1 ] ][ uv[ pt ][ 0 ] ][1] << 8) | self.image[ uv[ pt][ 1 ] ][ uv[ pt ][ 0 ] ][0]
#				pc_rgb[pt, 3] = color

				[ b, g, r ] = self.image[ int( uv[pt][ 1 ] ), int( uv[pt][ 0 ] )]
				pc_rgb[pt, 3] = 1
				pc_rgb[pt, 4:] = [ r/255.0, g/255.0, b/255.0 ] 
#				pcbgr.append( [ p[ 0 ], p[ 1 ], p[ 2 ], 0, r / 255.0, g / 255.0, b / 255.0 ] )

				cv_color = Color.get_rainbow_color(depth[ pt])
				cv2.circle(image_depth, (int(uv[pt][0]), int(uv[pt][1])), 1, cv_color, thickness = -1 )
				# cv2.rectangle(image_depth, (int(uv[pt][0])-halflenth, int(uv[pt][1])-halflenth), (int(uv[pt][0])+halflenth, int(uv[pt][1])+halflenth), cv_color ,thickness = -1 )


		try:
			self._pub_img_depth.publish( CvBridge().cv2_to_imgmsg( image_depth, 'bgr8' ) )

		except CvBridgeError as e:
			print(e)

		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "pandar"
		
		
#		fields = [PointField('x', 0, PointField.FLOAT32, 1),
#					PointField('y', 4, PointField.FLOAT32, 1),
#					PointField('z', 8, PointField.FLOAT32, 1),
#					PointField('rgb', 12, PointField.UINT32, 1)]

		fields = []
		fields.append( PointField( 'x', 0, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'y', 4, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'z', 8, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'intensity', 12, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'r', 16, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'g', 20, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'b', 24, PointField.FLOAT32, 1 ) )

		cloud_rgb = pc2.create_cloud(point_cloud_data.header, fields, pc_rgb)
		
		self.pub_points_rgb.publish( cloud_rgb )



if __name__ == '__main__':
	try:
		rospy.init_node("fusion1")   #初始化节点
		LidarImage()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

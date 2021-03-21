#!/usr/bin/env python
# -*- coding: UTF-8 -*- 

import rospy
from yolo_rect.msg import One_box, Boxes
from sensor_msgs.msg import Image


import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np

class yolo_distance_show():
    def __init__(self):
        self.yolo_img=0
        self.yolo_Boxes=0
        rospy.Subscriber('/usb_cam/image_rect_color', Image, self.callback_image)
        rospy.Subscriber("/yolo_distance", Boxes, self.callback_boxes)


    def callback_boxes(self,msg):
        if not isinstance(msg, Boxes):
            return
        self.yolo_Boxes=msg
        for box in self.yolo_Boxes.boxes:
            print(box)
            x1=int(box.x1)
            x2=int(box.x2)
            y1=int(box.y1)
            y2=int(box.y2)
            conf=box.conf
            label=box.label
            distance=box.distance
            self.plot_one(self.yolo_img,x1,x2,y1,y2,label,conf,distance)
        # cv2.rectangle(self.yolo_img,(300,300),(400,500),(0,255,0),2)

        cv2.imshow('source_img',self.yolo_img)
        cv2.waitKey(1)
        print("++++++++++++++++++++++++++++++++++++++++++")
    def callback_image(self,msg):
        if not isinstance(msg, Image):
            return
        self.yolo_img= np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # cv2.rectangle(self.yolo_img,(300,300),(400,500),(0,255,0),2)
        # cv2.imshow('source_img',self.yolo_img)
        # cv2.waitKey(1)


    def plot_one(self,img,x1,x2,y1,y2,label,conf,distance):
        c1=(int(x1),int(y1))
        c2=(int(x2),int(y2))
        cv2.rectangle(img, c1, c2,(0,255,255))
        # img = img[:, :, [2, 1, 0]]
        # label='类别'+label+'置信度'+(str(conf))[:4]+'距离'+(str(distance))[:4]+'m'
        label='class '+label+'distance '+(str(distance))[:4]+'m'
        if label:
            cv2.putText(img, label, (c1[0], c1[1]),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),1)


# def plot_one_box(x, img, color=None, label=None, line_thickness=None):
#     # Plots one bounding box on image img
#     tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
#     color = color or [random.randint(0, 255) for _ in range(3)]
#     c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
#     cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
#     if label:
#         tf = max(tl - 1, 1)  # font thickness
#         t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
#         c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
#         cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
#         cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

if __name__ == '__main__':
    nodeName = "rect_subscriber"
    rospy.init_node(nodeName)

    # 创建订阅者
    yolo_distance_show()

    rospy.spin()

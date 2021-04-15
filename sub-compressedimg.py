#! /usr/bin/env python
# import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
IMAGE_WIDTH=1241
IMAGE_HEIGHT=376
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import cv2
import numpy as np

ros_image=0

bridge = CvBridge()

def image_callback_1(ros_data):
    timestr = "%.6f" %  ros_data.header.stamp.to_sec()
    print("call back")
    #### direct conversion to CV2 ####
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    # np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    # cv2.imshow("cnm",image_np)
    # cv2.waitKey(33)
    publish_image(image_np)

def publish_image(imgdata):
    tmp=bridge.cv2_to_imgmsg(imgdata, "bgr8")
    tmp.header.stamp=rospy.Time.now()
    image_pubulish.publish(tmp)
    print("pub one")


if __name__ == '__main__':
    rospy.init_node('compressed_to_raw')
    image_topic_1 = "/usb_cam/image_raw/compressed"
    rospy.Subscriber(image_topic_1, CompressedImage, image_callback_1, queue_size=1, buff_size=52428800)

    image_pubulish=rospy.Publisher('/usb_cam/image_raw/',Image,queue_size=1)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    rospy.spin()

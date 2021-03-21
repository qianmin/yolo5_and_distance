#!/usr/bin/env python
# coding:utf-8

import rospy
from yolo_rect.msg import One_box, Boxes


def callback(msg):
    if not isinstance(msg, Boxes):
        return
    # print "name: %s" % msg.name
    # print "leader name: %s" % msg.leader.name
    # print "leader age: %d" % msg.leader.age
    # print "intro: %s" % msg.intro.data
    # print "location: {}".format(msg.location)

    for box in msg.boxes:
        print(box)

    print("--------------------")


if __name__ == '__main__':
    nodeName = "rect_subscriber"
    rospy.init_node(nodeName)

    # 创建订阅者
    topicName = "/yolo_distance"
    rospy.Subscriber(topicName, Boxes, callback)

    rospy.spin()

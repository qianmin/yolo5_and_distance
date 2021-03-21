/*
 * Created on Thu Apr 11 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Convert   //这里定义了Convert 类
{
public:
    Convert(ros::NodeHandle& nh, ros::NodeHandle& nh_local);  // 这是构造函数
private:
    void readParams();
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    /* ROS params*/
    ros::NodeHandle nh_, nh_local_;
    ros::Subscriber rslidar_sub_;
    ros::Publisher velodyne_pub_;     //在类的定义中，已经按照类型声明了变量
    /* class variable */
    std::string rslidar_topic_;  
    std::string velodyne_topic_;
};

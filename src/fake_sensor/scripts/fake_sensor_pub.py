#!/usr/bin/env python3

import rclpy
import math
import random

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg


def create_pointcloud(node):

    points = []

    # 随机生成一些障碍物
    for i in range(200):

        x = random.uniform(-5,5)
        y = random.uniform(-5,5)
        z = random.uniform(0,1)

        points.append([x,y,z])

    header = std_msgs.msg.Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = "map"

    cloud = pc2.create_cloud_xyz32(header, points)

    return cloud


def main():

    rclpy.init()
    node = rclpy.create_node("fake_sensor_node")

    odom_pub = node.create_publisher(Odometry, "/Odometry", 10)
    cloud_pub = node.create_publisher(PointCloud2, "/cloud_registered", 10)

    rate = node.create_rate(10)

    t = 0

    while rclpy.ok():

        # -------------------
        # 发布 odom
        # -------------------

        odom = Odometry()

        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = "map"

        odom.child_frame_id = "base_link"

        # 机器人绕圆运动
        x = 2 * math.cos(t)
        y = 2 * math.sin(t)

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0

        odom.pose.pose.orientation.w = 1.0

        odom_pub.publish(odom)

        # -------------------
        # 发布点云
        # -------------------

        cloud = create_pointcloud(node)

        cloud_pub.publish(cloud)

        t += 0.05

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

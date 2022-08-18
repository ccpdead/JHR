#!/usr/bin/env python3
#coding:utf-8

import rospy
import string
import math
import time
import sys

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from jhr_imu.msg import WT61CTTL

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
accel_factor = 9.806/256.0

seq = 0

def WT61CTTL_callback(data):
    yaw_deg = data.yaw
    yaw_deg = yaw_deg + imu_yaw_calibration

    yaw = yaw_deg*degrees2rad
    # pitch = float(data.pitch)*degrees2rad
    # roll  = float(data.roll)*degrees2rad
    pitch = 0.0
    roll = 0.0

    imuMsg.linear_acceleration.x = float(data.x_acc)*accel_factor
    imuMsg.linear_acceleration.y = float(data.y_acc)*accel_factor
    imuMsg.linear_acceleration.z = float(data.z_acc)*accel_factor

    imuMsg.angular_velocity.x = float(data.x_gyro)*degrees2rad
    imuMsg.angular_velocity.y = float(data.y_gyro)*degrees2rad
    imuMsg.angular_velocity.z = float(data.z_gyro)*degrees2rad

    q = quaternion_from_euler(roll, pitch, yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

######################################################################
    # Orientation covariance estimation
    #方向
    imuMsg.orientation_covariance = [
    1e6, 0 , 0,
    0 , 1e6, 0,
    0 , 0 , 1e-6
    ]

    # Angular velocity covariance estimation
    #速度
    imuMsg.angular_velocity_covariance = [
    1e6, 0 , 0,
    0 , 1e6, 0,
    0 , 0 , 1e-6
    ]
##################################################################
    # linear acceleration covariance estimation
    #加速度
    imuMsg.linear_acceleration_covariance = [
    -1 , 0 , 0,
    0 , 0, 0,
    0 , 0 , 0
    ]

    global seq
    imuMsg.header.stamp= rospy.Time().now()
    imuMsg.header.frame_id = "base_imu_link"
    imuMsg.header.seq = seq
    seq = seq + 1
    data_pub.publish(imuMsg)

    rate.sleep()

rospy.init_node("imu_node")
data_pub = rospy.Publisher("imu_data", Imu, queue_size=1)
rospy.Subscriber('WT61CTTL', WT61CTTL, WT61CTTL_callback)
imuMsg = Imu()

rate = rospy.Rate(100)
rospy.spin()

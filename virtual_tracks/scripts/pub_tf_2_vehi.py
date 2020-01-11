#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
import rospy
from std_msgs.msg import Float32
import pyproj
import tf
import math

publisher_1 = rospy.Publisher('gps_transform_1', NavSatFix, queue_size=10)
publisher_2 = rospy.Publisher('gps_transform_2', NavSatFix, queue_size=10)
rospy.init_node('include_gps_frame_id')
myproj = pyproj.Proj(proj = "utm", zone=32)
x_or,y_or = myproj(rospy.get_param("/lon_origin"), rospy.get_param("/lat_origin"))

def callback_1(msg):
   x,y = myproj(msg.longitude, msg.latitude)
   transformed_gps = msg
   transformed_gps.header.frame_id = "/car_frame_1"
   publisher_1.publish(transformed_gps)
   br = tf.TransformBroadcaster()
   yaw_msg = rospy.wait_for_message("/imu_data_1", Float32)
   br.sendTransform((x-x_or, y-y_or, 0),
                     tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_msg.data)),
                     rospy.Time.now(),
                     "/car_frame_1",
                     "/map")

def callback_2(msg):
   x,y = myproj(msg.longitude, msg.latitude)
   transformed_gps = msg
   transformed_gps.header.frame_id = "/car_frame_2"
   publisher_2.publish(transformed_gps)
   br = tf.TransformBroadcaster()
   yaw_msg = rospy.wait_for_message("/imu_data_2", Float32)
   br.sendTransform((x-x_or, y-y_or, 0),
                     tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_msg.data)),
                     rospy.Time.now(),
                     "/car_frame_2",
                     "/map")
   
sub_1 = rospy.Subscriber('/gps_data_1', NavSatFix , callback_1)
sub_2 = rospy.Subscriber('/gps_data_2', NavSatFix , callback_2)
rospy.spin()

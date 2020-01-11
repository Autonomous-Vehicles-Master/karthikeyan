#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
import rospy
import pyproj
import tf

publisher = rospy.Publisher('gps_transform', NavSatFix, queue_size=10)
rospy.init_node('include_gps_frame_id')
myproj = pyproj.Proj(proj = "utm", zone=32)
x_or,y_or = myproj(rospy.get_param("/lon_origin"), rospy.get_param("/lat_origin"))

def callback(msg):
   x,y = myproj(msg.longitude, msg.latitude)
   transformed_gps = msg
   transformed_gps.header.frame_id = "/base_link"
   publisher.publish(transformed_gps)
   br = tf.TransformBroadcaster()
   br.sendTransform((x-x_or, y-y_or, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/base_link",
                     "/map")
   
sub = rospy.Subscriber('/gps_data', NavSatFix , callback)
rospy.spin()

#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
import rospy
from lanelet2.projection import UtmProjector
import lanelet2
import pyproj
import pcl
import tf

publisher = rospy.Publisher('gps_transform', NavSatFix, queue_size=10)
rospy.init_node('gps_transform')
projector = UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
myproj = pyproj.Proj(proj = "utm", zone=32)
x_or,y_or = myproj(rospy.get_param("/lon_origin"), rospy.get_param("/lat_origin"))

def callback(msg):
   x,y = myproj(msg.longitude, msg.latitude)
   transformed_gps = msg
   transformed_gps.header.frame_id = "/base_link"
   x1 =  0.959077214940049 * x + 0.283144655167648 * y - 1502250.24640812
   y1 = -0.283144655167648 * x + 0.959077214940049 * y + 414244.447262422
   transformed_gps.longitude, transformed_gps.latitude = myproj(x1, y1, inverse = True)
   publisher.publish(transformed_gps)
   br = tf.TransformBroadcaster()
   br.sendTransform((x1-x_or, y1-y_or, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "/base_link",
                     "/map")
   
sub = rospy.Subscriber('/gps_data', NavSatFix , callback)
rospy.spin()

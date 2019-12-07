#!/usr/bin/env python

from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
import rospy
from lanelet2.projection import UtmProjector
import lanelet2
from lanelet2.core import LaneletLayer
from lanelet2.geometry import distance

publisher = rospy.Publisher('gps_marker', Marker, queue_size=10)
rospy.init_node('gps_marker')
projector = UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
file_ = rospy.get_param("/map_file_name")
map = lanelet2.io.load(file_, projector)

def callback(msg):
   global map
   point = projector.forward(lanelet2.core.GPSPoint(msg.latitude, msg.longitude))
   marker = Marker()
   marker.header.frame_id = "/map"
   marker.id = 0
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 3
   marker.scale.y = 3
   marker.scale.z = 3
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = point.x
   marker.pose.position.y = point.y
   marker.pose.position.z = 0 
   publisher.publish(marker)
   p2d = lanelet2.geometry.to2D(point)
   x = LaneletLayer.nearest(map.laneletLayer, p2d, 1)
   if distance(x[0], p2d) == 0:
   	print("You are in lanelet with ID: ", x[0].id)
   else:
	print("You are not inside a lanelet")
   
sub = rospy.Subscriber("/gps_transform", NavSatFix , callback)
rospy.spin()

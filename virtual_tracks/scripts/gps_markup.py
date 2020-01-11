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
map = lanelet2.io.load(rospy.get_param("/map_file_name"), projector)

def callback(msg):
   global map, previous_lanelet, previous_dist
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
   current_lanelet = LaneletLayer.nearest(map.laneletLayer, p2d, 1)
   current_dist = distance(current_lanelet[0], p2d)
   if current_lanelet[0] == previous_lanelet[0] and ((current_dist > 0 and previous_dist > 0) or (current_dist == previous_dist)):
	shall_i_print = False
   else:
	shall_i_print = True
   previous_lanelet[0] = current_lanelet[0]
   previous_dist = current_dist
   if shall_i_print:
       if current_dist == 0:
           print("You are in lanelet with ID: ", current_lanelet[0].id)
           for el in current_lanelet[0].rightOfWay():
		print "Yield untill there are no vehicles in lanelets :", [a.id for a in el.rightOfWayLanelets()]
       else:
	   print("You are not inside a lanelet")

previous_dist = 0
previous_lanelet = [0]
sub = rospy.Subscriber("/gps_transform", NavSatFix , callback)
rospy.spin()
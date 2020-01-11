#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import tf
import math
import lanelet2
from lanelet2.core import LaneletLayer
from lanelet2.geometry import distance
from visualization_msgs.msg import Marker

class Vehicle:
	def __init__(self):
		self.pub = rospy.Publisher('/gps_transform', NavSatFix, queue_size=10)
		#self.pub_text = rospy.Publisher('/information', Marker, queue_size=10)
		self.sub_gps = rospy.Subscriber('/gps_data_1', NavSatFix , self.callback_gps)
		self.sub_imu = rospy.Subscriber('/imu_data_1', Float32 , self.callback_imu)
		self.projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
		self.br = tf.TransformBroadcaster()
		self.pos = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin"))))
		self.map = lanelet2.io.load(rospy.get_param("/map_file_name"), self.projector)
		self.yaw = 0

	def callback_imu(self, msg):
		self.yaw = msg.data

	def callback_gps(self, msg):
	   	self.pos = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(msg.latitude, msg.longitude)))
	   	msg.header.frame_id = "/car_frame"
	   	self.pub.publish(msg)
	   	self.br.sendTransform((self.pos.x, self.pos.y, 0), tf.transformations.quaternion_from_euler(0, 0, math.radians(self.yaw)), rospy.Time.now(), "/car_frame", "/map")
 

if __name__ == '__main__':
	rospy.init_node('tf_navigation')
	Vehicle()
	rospy.spin()

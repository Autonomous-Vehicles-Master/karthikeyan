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
		print "loooooooooooooooooooooooooooooooo", rospy.get_param("~vehicle_no")
		self.pub = rospy.Publisher(str("/gps_transform_"+str(rospy.get_param("~vehicle_no"))), NavSatFix, queue_size=10)
		self.pub_text = rospy.Publisher(str("/information_"+str(rospy.get_param("~vehicle_no"))), Marker, queue_size=10)
		self.sub_gps = rospy.Subscriber(str("/gps_data_"+str(rospy.get_param("~vehicle_no"))), NavSatFix , self.callback_gps)
		self.sub_imu = rospy.Subscriber(str("/imu_data_"+str(rospy.get_param("~vehicle_no"))), Float32 , self.callback_imu)
		self.projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
		self.br = tf.TransformBroadcaster()
		self.pos = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin"))))
		self.map = lanelet2.io.load(rospy.get_param("/map_file_name"), self.projector)
		self.yaw = 0
		self.current_llet = (LaneletLayer.nearest(self.map.laneletLayer, self.pos, 1))[0]
		self.previous_llet = self.current_llet
		self.current_dist = distance(self.current_llet, self.pos)
		self.previous_dist = self.current_dist
		self.yield_lanlet = []
		self.current_nos = len([ll for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0])
		self.previous_nos = self.current_nos
		self.text = Marker()
		self.text.header.frame_id = "/map"
		self.text.scale.z = 1
		self.text.id = 0
		self.text.type = self.text.TEXT_VIEW_FACING
		self.text.action = self.text.ADD
		self.text.color.r = 0.65
		self.text.color.g = 0.24
		self.text.color.b = 0.25
		self.text.color.a = 1
		self.text.text = "Not inside a Lanelet"

	def callback_gps(self, msg):
		#self.yaw = (rospy.wait_for_message("/imu_data", Float32)).data
	   	self.pos = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(msg.latitude, msg.longitude)))
	   	msg.header.frame_id = str("/car_frame_"+str(rospy.get_param("~vehicle_no")))
	   	self.pub.publish(msg)
	   	self.br.sendTransform((self.pos.x, self.pos.y, 0), tf.transformations.quaternion_from_euler(0, 0, math.radians(self.yaw)), rospy.Time.now(), str("/car_frame_"+str(rospy.get_param("~vehicle_no"))), "/map")
		self.current_nos = len([ll for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0])
		if self.current_nos:
			self.current_llet = ([ll for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0])[0]
		else:
			self.current_llet = (LaneletLayer.nearest(self.map.laneletLayer, self.pos, 1))[0]
		self.current_dist = distance(self.current_llet, self.pos)
		self.yield_lanlet = []
		for el in self.current_llet.rightOfWay():
			self.yield_lanlet = self.yield_lanlet + [a.id for a in el.rightOfWayLanelets()]
		self.print_msg()
		self.previous_llet = self.current_llet
		self.previous_dist = self.current_dist
		self.previous_nos = self.current_nos
		self.text.pose.position.x = self.pos.x
		self.text.pose.position.y = self.pos.y
		self.text.pose.position.z = 2
		text = ""
		if len(self.yield_lanlet) and not self.current_dist:
			text = " Yield lanelets: " + str(self.yield_lanlet[0])
			for ll in self.yield_lanlet[1:]:
				text = text + " && " +str(ll)
		temp = self.text.text
		self.text.text = self.text.text + text
		self.pub_text.publish(self.text)
		self.text.text = temp

	def callback_imu(self, msg):
		self.yaw = msg.data

	def print_msg(self):
		if self.current_dist == 0 and self.previous_dist == 0 and self.current_nos ==1 and self.previous_nos == 1 and self.current_llet == self.previous_llet:
			# remain inside same lanelet
			return 0
		if self.current_dist == 0 and self.previous_dist == 0 and self.current_nos > 1 and self.previous_nos > 1 and self.current_nos == self.previous_nos:
			# remain in multiple lanelet
			return 0
		if self.current_nos == 0 and self.previous_nos == 0:
			# remain outside lanelet			
			return 0
		if len(self.yield_lanlet):
			print "Yield untill there are no vehicles in lanelets ",self.yield_lanlet 
		if self.current_dist == 0 and self.previous_dist == 0 and self.current_nos == 1 and self.previous_nos == 1 and not self.current_llet == self.previous_llet:
			# exactly one lane change
			print "1 You are in lanelet :", self.current_llet.id
			self.text.text = "Lanelet : " + str(self.current_llet.id)
			return 0
		if self.current_dist == 0 and not self.previous_dist == 0:
			# came inside a lanelet
			print "2 You are in lanelet :", self.current_llet.id
			self.text.text = "Lanelet : " + str(self.current_llet.id)
			return 0
		if self.current_dist == 0 and self.current_dist == 0 and self.previous_nos == 1 and self.current_nos > 1:
			# single to multiple
			x = [ll.id for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0]
			print "3 you are in one among the lanelets :", x
			text = "Lanelet : "
			for entry in x:
				text = text + str(entry) + " or "
			#print [ll.id for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0]
			self.text.text = text[:-3]
			return 0
		if self.current_dist == 0 and self.current_dist == 0 and self.previous_nos > 1 and self.current_nos == 1:
			# multiple to single
			print "4 You are in lanelet :", self.current_llet.id
			self.text.text = "Lanelet : " + str(self.current_llet.id)
			return 0
		if self.current_dist == 0 and self.current_dist == 0 and self.previous_nos > 1 and self.current_nos > 1:
			# multple to multiple
			x = [ll.id for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0]
			print "5 you are in one among the lanelets :" , x
			text = "Lanelet : "
			for entry in x:
				text = text + str(entry) + " or "
			#print [ll.id for ll in LaneletLayer.nearest(self.map.laneletLayer, self.pos, 4) if distance(ll, self.pos) == 0]
			self.text.text = text[:-3]
			return 0
		if self.previous_dist == 0 and not self.current_dist == 0:
			# went outside lanelet
			print "6 You are not inside lanelet"
			self.text.text = "Not inside a Lanelet"
			return 0
		print "none"
 

if __name__ == '__main__':
	rospy.init_node("~")
	Vehicle()
	rospy.spin()

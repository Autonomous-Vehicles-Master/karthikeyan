#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import tf
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import lanelet2
from lanelet2.geometry import distance, length, length2d, nearestPointAtDistance, project, to2D, to3D, distanceToCenterline2d, findNearest
from lanelet2.core import Lanelet,LineString3d, Point2d, Point3d, getId, LaneletMap, BoundingBox2d, BasicPoint2d, BasicPoint3d, ConstLineString3d, LaneletLayer
from lanelet2.routing import RoutingGraph
from lanelet2.projection import UtmProjector
import dynamic_reconfigure.client


class virt_cl:
	def __init__(self):
        	self.centerline = []

def length_point_list(point_list):
    	point_list_distance = 0
    	for i in range(len(point_list[:-1])):
        	point_list_distance += distance(point_list[i],point_list[i+1])
    	return point_list_distance

def get_a_marker(i_d, ps, pe):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.ARROW
	marker.action = marker.ADD
	marker.scale.x = 1
	marker.scale.y = 1
	marker.scale.z = 1
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 0.5
	marker.color.b = 0.0
   	marker.id = i_d
	marker.points = []
	marker.points.append(Point(ps.x,ps.y,0))
	marker.points.append(Point(pe.x,pe.y,0))
	return marker
	

class Vehicle:
	def __init__(self):
		self.target_lat = 48.784205
		self.target_long = 11.473232
		self.pub = rospy.Publisher(rospy.get_param("~navigation_marker"), MarkerArray, queue_size=10)
		self.sub_gps = rospy.Subscriber(rospy.get_param("~gps_data"), NavSatFix , self.callback_gps)
		self.projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
		self.vehicle_p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin"))))
		self.target_p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(self.target_lat, self.target_long)))
		self.map = lanelet2.io.load(rospy.get_param("/map_file_name"), self.projector)
		self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
		self.graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
		self.vehicle_lanelet = self.find_lanelet(self.vehicle_p2d)
		self.target_lanelet = self.find_lanelet(self.target_p2d)
		self.vehicle_point_in_lanelet_cl = self.point_in_lanelet(self.vehicle_lanelet, self.vehicle_p2d)
		self.target_point_in_lanelet_cl = self.point_in_lanelet(self.target_lanelet, self.target_p2d)
		self.route = self.graph.shortestPath(self.vehicle_lanelet, self.target_lanelet)
		self.marker_array = MarkerArray()
		self.client = dynamic_reconfigure.client.Client("track_server", timeout=30, config_callback=self.parameter_callback)
		self.route = self.graph.shortestPath(self.vehicle_lanelet, self.target_lanelet)

	def parameter_callback(self, config):
		self.target_lat = float(config.target_latitude)
		self.target_long = float(config.target_longitude)
		self.target_p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(self.target_lat, self.target_long)))
		self.target_lanelet = self.find_lanelet(self.target_p2d)
		self.target_point_in_lanelet_cl = self.point_in_lanelet(self.target_lanelet, self.target_p2d)
	
	def callback_gps(self, msg):
	   	self.vehicle_p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(msg.latitude, msg.longitude)))
		self.vehicle_lanelet = self.find_lanelet(self.vehicle_p2d)
		self.vehicle_point_in_lanelet_cl = self.point_in_lanelet(self.vehicle_lanelet, self.vehicle_p2d)
		self.route = self.graph.shortestPath(self.vehicle_lanelet, self.target_lanelet)
		self.show_path()
	
	def point_in_lanelet(self, lanelet, p2d):
        	point = project(to2D(lanelet.centerline), p2d)
        	return BasicPoint3d(point.x,point.y)

	def find_lanelet(self, p2d):
        	lanelet_list = [lanelet for lanelet in LaneletLayer.nearest(self.map.laneletLayer, p2d, 4)]
        	closest_lanelet = lanelet_list[0]
        	closest_distance = distanceToCenterline2d(closest_lanelet, p2d)
        	for lanelet in lanelet_list:
            		if distanceToCenterline2d(lanelet, p2d) < closest_distance:
				closest_lanelet = lanelet
               		 	closest_distance = distanceToCenterline2d(lanelet, p2d)
        	return closest_lanelet

	def show_path(self):
		self.marker_array.markers = []
		i = 0
		lanelet_list = [lanelet for lanelet in self.route]

		edges = [node for node in lanelet_list[0].centerline]
		flag = False
		for index,point in enumerate(edges[:-1]):
			if distance(point, self.vehicle_point_in_lanelet_cl) < distance(point, edges[index+1]):
				flag = True				
				self.marker_array.markers.append(get_a_marker(i, self.vehicle_point_in_lanelet_cl, edges[index+1]))
				i = i + 1
				for index_1,point_1 in enumerate(edges[index+1:-1]):
					self.marker_array.markers.append(get_a_marker(i, point_1, edges[index_1+index+2]))
					i = i + 1
			if flag:
				break


		for lanelet in lanelet_list[1:-1]:
			edges = [node for node in lanelet.centerline]
			for index,point in enumerate(edges[:-1]):
				self.marker_array.markers.append(get_a_marker(i, point, edges[index+1]))
				i = i + 1

		edges = [node for node in lanelet_list[-1].centerline]
		for index,point in enumerate(edges[:-1]):
			if distance(point, self.target_point_in_lanelet_cl) < distance(point, edges[index+1]):
				self.marker_array.markers.append(get_a_marker(i, point, self.target_point_in_lanelet_cl))
				break
			self.marker_array.markers.append(get_a_marker(i, point, edges[index+1]))
			i = i + 1

		self.pub.publish(del_marker_array)
		self.pub.publish(self.marker_array)
							
		

if __name__ == '__main__':
	rospy.init_node("~")
	del_marker_array = MarkerArray()
	del_marker = Marker()
	del_marker.action = del_marker.DELETEALL
	del_marker.header.frame_id = "/map"
	del_marker_array.markers = [del_marker]
	Vehicle()
	rospy.spin()








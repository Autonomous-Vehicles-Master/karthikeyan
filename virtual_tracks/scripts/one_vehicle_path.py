#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import tf
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import lanelet2
from lanelet2.geometry import distance, length, project, to2D, distanceToCenterline2d
from lanelet2.core import Lanelet, Point3d, BasicPoint3d, LaneletLayer, GPSPoint
from lanelet2.routing import RoutingGraph
from lanelet2.projection import UtmProjector
from lanelet2.io import load, Origin
from lanelet2.traffic_rules import create, Locations, Participants

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
	marker.color.g = 1.0
	marker.color.b = 0.0
   	marker.id = i_d
	marker.points = []
	marker.points.append(Point(ps.x,ps.y,0))
	marker.points.append(Point(pe.x,pe.y,0))
	return marker
	

class Vehicle:
	def __init__(self):
		self.spd = 0
		self.marker_array = MarkerArray()
		self.pub = rospy.Publisher(str("path_marker_"+str(rospy.get_param("~vehicle_no"))), MarkerArray, queue_size=10)
		self.sub_gps = rospy.Subscriber(str("gps_data_"+str(rospy.get_param("~vehicle_no"))), NavSatFix , self.callback_gps)
		self.sub_spd = rospy.Subscriber(str("speed_"+str(rospy.get_param("~vehicle_no"))), Float32 , self.callback_spd)
		self.projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin")))
		self.p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(rospy.get_param("/lat_origin"), rospy.get_param("/lon_origin"))))
		self.map = lanelet2.io.load(rospy.get_param("/map_file_name"), self.projector)
		self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
		self.graph = lanelet2.routing.RoutingGraph(self.map, self.traffic_rules)
		self.lanelet = self.find_lanelet()
		self.vehicle_point_in_lanelet_cl = self.vehicle_point_in_lanelet()
		self.routes_lines = self.find_routes_by_time(3)
		
	def callback_gps(self, msg):
	   	self.p2d = lanelet2.geometry.to2D(self.projector.forward(lanelet2.core.GPSPoint(msg.latitude, msg.longitude)))
		self.lanelet = self.find_lanelet()
		if distance(self.lanelet, self.p2d):
			self.pub.publish(del_marker_array)
			return 0
		self.vehicle_point_in_lanelet_cl = self.vehicle_point_in_lanelet()
		self.routes_lines = self.find_routes_by_time(3)
		self.show_path()

	def callback_spd(self, msg):
	   	self.spd = msg.data
	
	def vehicle_point_in_lanelet(self):
        	point = project(to2D(self.lanelet.centerline), self.p2d)
        	return BasicPoint3d(point.x,point.y)

	def find_lanelet(self):
        	lanelet_list = [lanelet for lanelet in LaneletLayer.nearest(self.map.laneletLayer, self.p2d, 4)]
        	closest_lanelet = lanelet_list[0]
        	closest_distance = distanceToCenterline2d(closest_lanelet, self.p2d)
        	for lanelet in lanelet_list:
            		if distanceToCenterline2d(lanelet, self.p2d) < closest_distance:
				closest_lanelet = lanelet
               		 	closest_distance = distanceToCenterline2d(lanelet, self.p2d)
        	return closest_lanelet

	def find_routes_by_time(self, dist):
		dist = self.spd * dist
        	route_start = self.find_start_route(dist)
        	routes = [[]]
        	s = length_point_list(route_start.centerline)
        	for route in routes:
            		flag = False
            		while True:
                		if sum([length(lanelet_in_route.centerline) for lanelet_in_route in route])+s >= dist:
                    			break
                		if len(route) == 0:
                    			passable = RoutingGraph.following(self.graph, self.lanelet)
                		else:
                    			passable = RoutingGraph.following(self.graph, route[-1])
                		if len(passable) == 0:
                    			flag = True
                    			break
                		elif len(passable) > 1:
                    			for lanelet in passable[1:]:
                        			routes.append(list(route))
                        			routes[-1].append(lanelet)
                		route.append(passable[0])
            		if(len(route) and not flag):
                		route[-1] = self.find_end_route(route[-1], dist-sum([length(lanelet_in_route.centerline) for lanelet_in_route in route[:-1]])-s)
        	for route in routes:
			route.insert(0,route_start)
        	return routes

    	def find_start_route(self, dist):
        	route_start = virt_cl()
        	route_start.centerline.append(self.vehicle_point_in_lanelet_cl)
        	for i in range(len(self.lanelet.centerline)-1):
            		if distance(self.lanelet.centerline[i],self.vehicle_point_in_lanelet_cl)<distance(self.lanelet.centerline[i],self.lanelet.centerline[i+1]):
                		for j in range(i+1,len(self.lanelet.centerline)):
                    			route_start.centerline.append(self.lanelet.centerline[j])
                    			if length_point_list(route_start.centerline)>dist:
                        			l1 = dist-length_point_list(route_start.centerline[:-1])
                        			l2 = distance(route_start.centerline[-1],route_start.centerline[-2])
                        			x1 = route_start.centerline[-2].x + (l1/l2)*(route_start.centerline[-1].x-route_start.centerline[-2].x)
                        			y1 = route_start.centerline[-2].y + (l1/l2)*(route_start.centerline[-1].y-route_start.centerline[-2].y)
                        			route_start.centerline[-1] = BasicPoint3d(x1, y1)
                        			break
                		break
        	return route_start

    	def find_end_route(self, end_lanelet, end_dist_reqd):
        	route_end = virt_cl()
        	for point in end_lanelet.centerline:
            		route_end.centerline.append(point)
            		if length_point_list(route_end.centerline)>end_dist_reqd:
                		break
        	l1 = end_dist_reqd-length_point_list(route_end.centerline[:-1])
        	l2 = distance(route_end.centerline[-1],route_end.centerline[-2])
        	x1 = route_end.centerline[-2].x + (l1/l2)*(route_end.centerline[-1].x-route_end.centerline[-2].x)
        	y1 = route_end.centerline[-2].y + (l1/l2)*(route_end.centerline[-1].y-route_end.centerline[-2].y)
        	route_end.centerline[-1] = BasicPoint3d(x1, y1)
        	return route_end

	def show_path(self):
		self.marker_array.markers = []
		i = 0
		for route in self.routes_lines:
			for lanelet in route:
				edges = [node for node in lanelet.centerline]
				for index,point in enumerate(edges[:-1]):
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








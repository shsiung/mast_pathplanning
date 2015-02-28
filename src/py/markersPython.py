#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
from geometry_msgs.msg import Point
from tsp_approx import TravellingSalesmanMST
from min_hamiltonian import MinHamiltonian

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=10)

rospy.init_node('hamiltonian')

marker = Marker()
marker.header.frame_id = "/map"
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.id = 0
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.2

startPoint_mk = Marker()
startPoint_mk.header.frame_id = "/map"
startPoint_mk.type = Marker.SPHERE
startPoint_mk.action = Marker.ADD
startPoint_mk.pose.orientation.w = 1.0
startPoint_mk.id = 2
startPoint_mk.color.a = 1.0
startPoint_mk.color.r = 0.0
startPoint_mk.color.g = 1.0
startPoint_mk.color.b = 0.0
startPoint_mk.scale.x = 0.5
startPoint_mk.scale.y = 0.5
startPoint_mk.scale.z = 0.5

endPoint_mk = Marker()
endPoint_mk.header.frame_id = "/map"
endPoint_mk.type = Marker.SPHERE
endPoint_mk.action = Marker.ADD
endPoint_mk.pose.orientation.w = 1.0
endPoint_mk.id = 3
endPoint_mk.color.a = 1.0
endPoint_mk.color.r = 1.0
endPoint_mk.color.g = 0.0
endPoint_mk.color.b = 1.0
endPoint_mk.scale.x = 0.5
endPoint_mk.scale.y = 0.5
endPoint_mk.scale.z = 0.5

marker_link = Marker()
marker_link.header.frame_id = "/map"
marker_link.type = Marker.LINE_STRIP
marker_link.action = Marker.ADD
marker_link.pose.orientation.w = 1.0
marker_link.id = 1
marker_link.color.a=1.0
marker_link.color.b=1.0
marker_link.scale.x=0.1

count = 0
MARKERS_MAX = 30
BOUNDARY = 10;

# Using the travelling salesman algs to find a traversal
def tsp():
    randX = np.random.random_sample(MARKERS_MAX)*BOUNDARY*2-BOUNDARY
    randY = np.random.random_sample(MARKERS_MAX)*BOUNDARY*2-BOUNDARY
    randZ = np.random.random_sample(MARKERS_MAX)*BOUNDARY/2

    coords = np.matrix.transpose(np.array([randX, randY, randZ]))
    startPoint = coords[0]
    endPoint = coords[-1]
    mhp = MinHamiltonian(startPoint, coords[1:-1], endPoint)
    path = mhp.minPath()
    # tsp = TravellingSalesmanMST(startPoint, coords[1:])
    # coordsT = np.matrix.transpose(np.array(coords))

    # path = tsp.tspPath()
    # print path
    # print tsp.prim.mstEdges
    pt_generation(path, len(path))

# Generate a list of markers
def pt_generation(coords, path_len):
    startPoint_mk.pose.position.x = coords[0][0]
    startPoint_mk.pose.position.y = coords[0][1]
    startPoint_mk.pose.position.z = coords[0][2]

    endPoint_mk.pose.position.x = coords[path_len-1][0]
    endPoint_mk.pose.position.y = coords[path_len-1][1]
    endPoint_mk.pose.position.z = coords[path_len-1][2]


    for i in range(path_len):
        p = Point()
        p.x = coords[i][0]
        p.y = coords[i][1]
        p.z = coords[i][2]

        marker.points.append(p)
        marker_link.points.append(p)

# Publish the MarkerArray
if __name__== "__main__":
    tsp()
    #print markerArray
    while not rospy.is_shutdown():
    	publisher.publish(marker)
        publisher.publish(marker_link)
        publisher.publish(startPoint_mk)
        publisher.publish(endPoint_mk)
        # publisher.publish(markerLine)
	rospy.sleep(0.01)


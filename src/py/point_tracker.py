#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import cos,sin,pi
from tf.broadcaster import TransformBroadcaster
import tf
import rospy
import signal
import time
import numpy as np
from scipy import linalg
from simulated_plane import Sim_Plane
import matplotlib.pyplot as plt

class Path_Manager:

	# Constructor
	def __init__(self,path):
		self.path = path
		self.start = 0;
		self.goal = 1;
		self.nxt = 2;

	def get_direction(self,pos):
		r = self.path[self.start];
		
		q_cur = (np.array(self.path[self.goal]) - np.array(self.path[self.start]))/ \
		  linalg.norm(np.array(self.path[self.goal]) - np.array(self.path[self.start]))

		q_nxt = (np.array(self.path[self.nxt]) - np.array(self.path[self.goal]))/ \
		  linalg.norm(np.array(self.path[self.nxt]) - np.array(self.path[self.goal]))

		n = (np.array(q_nxt)-np.array(q_nxt))/ \
		  linalg.norm((np.array(q_nxt)-np.array(q_nxt)))

		# if np.dot((np.array(pos) - np.array(r)),np.array(n)) >= 0:
		if linalg.norm(np.array(self.path[self.goal]) - np.array(pos)) < 0.2:
			self.start += 1
			self.goal += 1
			self.nxt += 1
			print "Switch to point: " + str(self.start)

		return (r,q_cur)

	def straight_line_following(self, start, des_dir, pos, heading):
		heading_correct = heading*pi/180 - pi/2
		x_q = np.arctan2(des_dir[1],des_dir[0])
		k_path = 0.5

		while x_q - heading < -pi:
			x_q += 2*pi
		while x_q - heading > pi:
			x_q -= 2*pi
		e_py = -sin(x_q*(pos[1]-start[1]))+cos(x_q*(pos[0]-start[0]))

		x_ca = e_py*0.5
		x_c = x_q - x_ca*2/pi*np.arctan(k_path*e_py)
		return x_c

if __name__== "__main__":
	numV = 10
	randX = np.random.random_sample(numV)*10 - 5
	randY = np.random.random_sample(numV)*10 - 5
	randZ = np.random.random_sample(numV)*10

	coords = np.matrix.transpose(np.array([randX, randY, randZ]))

	pm = Path_Manager(coords)
	plane = Sim_Plane()
	plane.x_dot = 1.0;

	plane.x = coords[0][0]
	plane.y = coords[0][1]
	plane.z = coords[0][2]

	print coords

	while not rospy.is_shutdown():
        	plane.get_plane_state()
        	# plane.update_state()

		(r,q) = pm.get_direction(plane.get_plane_state())
		des_r = pm.straight_line_following(r,q,plane.get_plane_state(),plane.yaw)
		print q
		plane.yaw = des_r

		plane.x += q[0] * plane.deltaT * 3
		plane.y += q[1] * plane.deltaT* 3
		plane.z += q[2] * plane.deltaT* 3
		plane.broadcast_state()

		print plane.get_plane_state()

		rospy.sleep(plane.deltaT)



	# plt.clf()
	# start
	# plt.plot(r[0], q+r, 'o', lw=1)
	# # plt.plot(r, q+r, 'k-', lw=1)
	# plt.show()


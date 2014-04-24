#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy
import os;

# Import the messages we're interested in sending and receiving
# from sensor_msgs.msg import Imu, Range
# from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist #for cmd_vel
from std_msgs.msg import Empty #for takeoff land and reset
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
# from diagnostic_msgs.msg import KeyValue # added for State Handling

from ardrone_control.srv import Signal #for signal messages


from Quadrotor import Quadrotor
# from BasicObject import ControllerState
from Signals import SignalResponse
from ROS import ROS_Object 


import tf
from math import pi, cos, log 

A = 0.1
phi_0 = 0.
f0 = 0.01
f1 = 20.
t1 = 100.
k_lin = (f1 - f0)/t1
k_exp =  (f1/f0) ** (1/t1)

Ts = 0.001


def chirp(t, method = 'linear'):
	if method == 'linear':
		f = f0 * t  + k_lin/2.0 * t ** 2
	elif method == 'exponential':
		f = f0 * (k_exp ** t - 1)/log(k_exp)

	return A * cos( phi_0 + 2 * pi *  f  )

class ROS_Handler(Quadrotor, ROS_Object, object):
	"""docstring for ROS_Handler:
	Subscribed to:
		ardrone/navdata -> reads state of the ardrone

	Publishes to:
		ardrone/land
		ardrone/takeoff
		ardrone/reset
		ardrone/controller/state -> communicates to the controller its state

		ardrone/trajectory -> Prints in an Odometry msg the desired trajectory 
			It is called by a Timer 
			trajectory data is taken from a self.quadrotor object instance 

	"""
	def __init__(self, **kwargs):
		super(ROS_Handler, self).__init__()

		self.publisher.update(
			land = rospy.Publisher('ardrone/land', Empty, latch = True),
			takeoff = rospy.Publisher('ardrone/takeoff', Empty, latch = True),
			reset = rospy.Publisher('ardrone/reset', Empty, latch = True),
			cmd_vel = rospy.Publisher('cmd_vel', Twist),
			)		

		self.subscriber.update(
			ardrone_state = rospy.Subscriber('ardrone/navdata', Navdata, callback = self.RecieveNavdata),
			)

		self.timer.update(
			timer = rospy.Timer(rospy.Duration( Ts ), self.Talk, oneshot=False),
			)

		self.signal = dict( x = False, y = False, z = False, yaw = False)

		self.msg = Twist()

		self.t0 = rospy.get_time();
		self.t = rospy.get_time();

	def RecieveNavdata( self, data ):
		self.set_state(data.state)

	def Talk(self, time):
		# time.current_expected, time.current_real, time.last_duration, time.last_expected, time.last_real

		self.t = rospy.get_time() - self.t0

		if self.signal['x']:
			self.msg.linear.x = chirp(self.t, method ='exponential')
			self.msg.linear.y = 0
			self.msg.linear.z = 0
			self.msg.angular.z = 0
		elif self.signal['y']:
			self.msg.linear.x = 0
			self.msg.linear.y = chirp(self.t, method ='exponential')
			self.msg.linear.z = 0
			self.msg.angular.z = 0
		elif self.signal['z']:
			self.msg.linear.x = 0
			self.msg.linear.y = 0
			self.msg.linear.z = chirp(self.t, method ='exponential')
			self.msg.angular.z = 0
		elif self.signal['yaw']:
			self.msg.linear.x = 0
			self.msg.linear.y = 0
			self.msg.linear.z = 0
			self.msg.angular.z = chirp(self.t, method ='exponential')
		else:
			self.msg.linear.x = 0
			self.msg.linear.y = 0
			self.msg.linear.z = 0
			self.msg.angular.z = 0


		self.publisher['cmd_vel'].publish( self.msg ) #always publish so last command makes it stop

	def TakeOff(self, *args):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.state == 'Landed'):
			self.publisher['takeoff'].publish()
		else:
			pass

	def Reset(self, *args):
		self.publisher['reset'].publish()

	def Land(self, *args):
		self.publisher['land'].publish()
	
	def Chirp(self, direction):
		self.t0 = rospy.get_time()
		self.t = rospy.get_time() - self.t0

		for key in self.signal.keys():
			if key == direction:
				self.signal[key] = True
			else:
				self.signal[key] = False

def main():
	rospy.init_node('ChirpCommand', anonymous = True)
	ros_handler = ROS_Handler( )
	
	ros_handler.TakeOff() 
	while not ros_handler.state == 'Flying':
		rospy.sleep(0.1)
		ros_handler.TakeOff() 
	else:
		rospy.sleep(3.0)
		print "X - Chirp"
		ros_handler.Chirp('x')

	while ros_handler.t < t1:
		rospy.sleep(1.)
	else:
		ros_handler.Chirp(None)
		rospy.sleep(3.0)
		print "Y - Chirp"
		ros_handler.Chirp('y')

	while ros_handler.t < t1:
		rospy.sleep(1.)
	else:
		ros_handler.Chirp(None)
		rospy.sleep(3.0)
		print "Z - Chirp"
		ros_handler.Chirp('z')

	while ros_handler.t < t1:
		rospy.sleep(1.)
	else:
		ros_handler.Chirp(None)
		rospy.sleep(3.0)
		print "Yaw - Chirp"
		ros_handler.Chirp('yaw')

	while ros_handler.t < t1:
		rospy.sleep(1.)
	else:
		print "Done"
		ros_handler.Chirp(None)

		os.system("rosnode kill --all")

		


    

if __name__ == "__main__": main()

#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

import os;


# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from diagnostic_msgs.msg import KeyValue # added for State Handling

from ardrone_control.srv import Signal


from Quadrotor import Quadrotor
from BasicObject import ControllerState
from Signals import SignalResponse
from ROS import ROS_Object 


import tf
from math import pi, sin, cos 

Command_Time = 0.005;
IMU_Period = 0.01;
Total_Time = 20.0;

def f_x(t):
	return cos(2 * pi * t / Total_Time )
def f_y(t):
	return 0.5*sin( 4 * pi * t / Total_Time)
def f_z(t):
	return 1 - cos( 3 * pi * t / Total_Time )
def f_yaw(t):
	return 4.5 * pi * t / Total_Time

def df_x(t):
	#return 0
	return - 2 * pi / Total_Time * sin( 2 * pi * t / Total_Time )
def df_y(t):
	#return 0
	return 2 * pi / Total_Time * cos( 4 * pi * t / Total_Time)
def df_z(t):
	#return 0
	return 3 * pi / Total_Time * sin( 3 * pi * t / Total_Time)
def df_yaw(t):
	#return 0
	return 4.5 * pi / Total_Time


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

		self.t = 0.0;
		self.Ts = kwargs.get('Command_Time', IMU_Period)

		self.name = kwargs.get('name', "/goal")

		self.publisher.update(
			land = rospy.Publisher('/ardrone/land', Empty, latch = True),
			takeoff = rospy.Publisher('/ardrone/takeoff', Empty, latch = True),
			reset = rospy.Publisher('/ardrone/reset', Empty, latch = True),
			controller_state = rospy.Publisher('/ardrone/controller/state', KeyValue, latch = True),
			trajectory = rospy.Publisher('/ardrone/trajectory', Odometry) )

		self.subscriber.update(
			ardrone_state = rospy.Subscriber('/ardrone/navdata', Navdata, callback = self.RecieveNavdata),
			)

		self.tf_broadcaster.update( 
			goal_tf = tf.TransformBroadcaster() 
			)

		self.timer.update(
			trajectory_timer = None,
			)

	def StartTrajectory(self):
		self.timer.update( trajectory_timer = rospy.Timer(rospy.Duration( self.Ts ), self.TalkTrajectory, oneshot=False) )
	def StopTrajectory(self):
		self.timer['trajectory_timer'].shutdown()
		self.timer.update( trajectory_timer = rospy.Timer(rospy.Duration( self.Ts ), self.TalkRegulator, oneshot=False) )

	def RecieveNavdata( self, data ):
		self.set_state(data.state)

	def TalkRegulator(self, time):

		msgs = Odometry( )
		msgs.header.stamp = rospy.Time.now()
		msgs.header.frame_id = "/nav"
		msgs.child_frame_id = "/goal" 

		# position
		msgs.pose.pose.position.x = f_x(self.t)
		msgs.pose.pose.position.y = f_y(self.t)
		msgs.pose.pose.position.z = f_z(self.t)

		# orientation
		orientation = tf.transformations.quaternion_from_euler(0, 0, f_yaw(self.t))
		msgs.pose.pose.orientation.x = orientation[0]
		msgs.pose.pose.orientation.y = orientation[1]
		msgs.pose.pose.orientation.z = orientation[2]
		msgs.pose.pose.orientation.w = orientation[3]
		
		# linear velocity
		msgs.twist.twist.linear.x = 0
		msgs.twist.twist.linear.y = 0
		msgs.twist.twist.linear.z = 0

		# angular velocity
		msgs.twist.twist.angular.x = 0
		msgs.twist.twist.angular.y = 0
		msgs.twist.twist.angular.z = 0

		self.publisher['trajectory'].publish(msgs)

		self.tf_broadcaster['goal_tf'].sendTransform( 
			(msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)

	def TalkTrajectory( self, time ):
		""" Publishes the Goal Point for the Controller """

		msgs = Odometry( )
		msgs.header.stamp = rospy.Time.now()
		msgs.header.frame_id = "/nav"
		msgs.child_frame_id = "/goal" 

		# position
		msgs.pose.pose.position.x = f_x(self.t)
		msgs.pose.pose.position.y = f_y(self.t)
		msgs.pose.pose.position.z = f_z(self.t)

		# orientation
		orientation = tf.transformations.quaternion_from_euler(0, 0, f_yaw(self.t))
		msgs.pose.pose.orientation.x = orientation[0]
		msgs.pose.pose.orientation.y = orientation[1]
		msgs.pose.pose.orientation.z = orientation[2]
		msgs.pose.pose.orientation.w = orientation[3]
		
		# linear velocity
		msgs.twist.twist.linear.x = df_x(self.t)
		msgs.twist.twist.linear.y = df_y(self.t)
		msgs.twist.twist.linear.z = df_z(self.t)

		# angular velocity
		msgs.twist.twist.angular.x = 0
		msgs.twist.twist.angular.y = 0
		msgs.twist.twist.angular.z = df_yaw(self.t)

		self.publisher['trajectory'].publish(msgs)

		self.tf_broadcaster['goal_tf'].sendTransform( 
			(msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)

		self.t += self.Ts 

	def TakeOff(self, *args):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		print self.state
		if(self.state == 'Landed'):
			self.publisher['takeoff'].publish()
			print "Taking Off!"
		else:
			print "Drone is not landed"

	def Reset(self, *args):
		self.ControlOff()
		self.SignalOff()

		self.publisher['reset'].publish()
		print "Reset"

	def Land(self, *args):
		self.ControlOff()
		self.SignalOff()
		self.publisher['land'].publish()
		print "Landing"
	
	def ControlOff(self, *args):
		self.change_controller_state('Off')

	def ControlOn(self, *args):
		if self.state == 'Flying' or self.state == 'Hovering':
			self.change_controller_state('On')

	def change_controller_state(self, state_new_name):
		new_state = KeyValue()

		if state_new_name in ControllerState.MAP:
			new_state.key = state_new_name	
		else: 
			new_state.key = 'Unknown'

		new_state.value = str( ControllerState.MAP.index( new_state.key ) )

		self.publisher['controller_state'].publish(new_state)

		print "Controller New State is:", new_state.key
	
def main():
	rospy.init_node('StateHandler', anonymous = True)
	ros_handler = ROS_Handler(Command_Time = IMU_Period)
	
	ros_handler.TakeOff() 
	while not ros_handler.state == 'Flying':
		rospy.sleep(1.)
		ros_handler.TakeOff() 
	else:
		ros_handler.ControlOn()
		ros_handler.StartTrajectory()

	while ros_handler.t < Total_Time:
		continue;
	else: 
		ros_handler.StopTrajectory()

	rospy.sleep(20.)

	os.system("rosnode kill --all")

	rospy.spin()
		


    

if __name__ == "__main__": main()
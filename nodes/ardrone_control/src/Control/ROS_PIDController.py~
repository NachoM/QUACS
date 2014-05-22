#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Vector3Stamped # for trajectory and space Estimation
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from diagnostic_msgs.msg import KeyValue # added for State Handling

import Controller 
from ROS import ROS_Object 
from BasicObject import ControllerState

from math import pi, cos, sin 
import tf 

# Some Constants
Command_Time = 0.01;
g = 9.81;

class DroneController(object):
	"""docstring for DroneController

	There is an array of position controllers and an array of orientation controllers 
	There is a yaw property that maps from local to world coordinates. A tf is planned. 

	"""
	def __init__(self, **kwargs):
		super(DroneController, self).__init__()
		Gains = rospy.get_param('Gains', dict( X = dict(P = 1.0, I = 0.0, D = 0.0), 
			Y = dict(P = 1.0, I = 0.0, D = 0.0),
			Z = dict(P = 1.0, I = 0.0, D = 0.0),
			Yaw = dict(P = 1.0, I = 0.0, D = 0.0) ) )

		self.position_control = dict( 
			x = Controller.PID_Controller( Ts = Command_Time, Kp = Gains['X']['P'], Ki = Gains['X']['I'], Kd = Gains['X']['D'] ), 
			y = Controller.PID_Controller( Ts = Command_Time, Kp = Gains['Y']['P'], Ki = Gains['Y']['I'], Kd = Gains['Y']['D'] ), 
			z = Controller.PID_Controller( Ts = Command_Time, Kp = Gains['Z']['P'], Ki = Gains['Z']['I'], Kd = Gains['Z']['D'] ) )

		self.orientation_control = dict( z = Controller.PID_Controller( Ts = Command_Time, 
			Kp = Gains['Yaw']['P'], Ki = Gains['Yaw']['I'], Kd = Gains['Yaw']['D'] , periodic = True) )
		
		if not hasattr(self, 'properties'):
			self.properties = dict()


		self.yaw = kwargs.get('yaw', 0)
		self.state = kwargs.get('state', ControllerState())

	@property 
	def yaw(self):
		return self.properties.get('yaw', 0)
	@yaw.setter 
	def yaw(self, yaw):
		self.properties['yaw'] = yaw
	@yaw.deleter
	def yaw(self):
		del self.properties['yaw']
	
	# Object Properties
	@property 
	def state(self):
		try:
			return getattr(self.properties.get('state'), 'state')
		except AttributeError:
			return None
	@state.setter
	def state(self, state):
		if self.state is not None:
			self.properties['state'].set_state(state)
		else:
			self.properties['state'] = state
	@state.deleter
	def state(self):
		del self.properties['state']

class ROS_Handler(DroneController, ROS_Object, object):
	"""docstring for ROS_Handler

	Subscribed to:
		ardrone/sensorfusion/navdata -> reads estimation of the ardrone state 
		ardrone/trajectory -> reads desired set point  
		controller/state -> reads the controller state 

	Publishes to:
		ardrone/cmd_vel -> Actuates on ardrone velocity only in a State 
			It is called by a Timer
			Transformed to local coordinates with a self.yaw properties -> tf will be used.   

	Angles_MAP is just a map from [x,y,z] keys to the index of tf.transformations.euler_from_quaternion 
	"""

	Angles_MAP = dict( 
		x = 0, 
		y = 1, 
		z = 2)
	def __init__(self, **kwargs):
		super(ROS_Handler, self).__init__()
		
		self.subscriber.update(
			controller_state = rospy.Subscriber('ardrone/controller/state', KeyValue, callback = self.RecieveState),
			state_estimation = rospy.Subscriber('ardrone/sensorfusion/navdata', Odometry, callback = self.RecieveOdometry, callback_args = 'set_input' ),
			set_point = rospy.Subscriber('ardrone/trajectory', Odometry, callback = self.RecieveOdometry, callback_args = 'change_set_point' )
			)

		self.publisher.update( 
			command_velocity = rospy.Publisher('cmd_vel', Twist)
			)

		self.timer.update( 
			actuation_timer = rospy.Timer(rospy.Duration( Command_Time ), self.Actuate, oneshot=False) 
			)
		
		self.tfListener = tf.TransformListener()
		
		# self.angles_map = dict(x = 0, y = 1 , z = 2)
		self.my_tf_prefix = rospy.get_param('tf_prefix')

	def RecieveOdometry( self, data , method):

		point_data = Vector3Stamped()
		point_data.header.frame_id = data.header.frame_id

		#transform position
		point_data.vector = data.pose.pose.position 
		data.pose.pose.position = self.tfListener.transformVector3(self.my_tf_prefix+'/drone_local', point_data).vector
		#transform velocity
		point_data.vector = data.twist.twist.linear 
		data.twist.twist.linear = self.tfListener.transformVector3(self.my_tf_prefix+'/drone_local', point_data).vector



		for key in self.position_control.keys():
			getattr(self.position_control[key], method)( position = getattr(data.pose.pose.position, key), velocity = getattr(data.twist.twist.linear, key ) )

		angles = tf.transformations.euler_from_quaternion( [data.pose.pose.orientation.x , data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] )
		for key in self.orientation_control.keys():
			getattr(self.orientation_control[key], method) ( position = angles[ self.Angles_MAP[key] ] , velocity = getattr(data.twist.twist.angular, key ) )

		if method == 'set_input':
			self.yaw = angles[ self.Angles_MAP['z'] ]


	def Actuate(self, time):
		if self.state == 'On':
			twist = Twist()
			for key in self.position_control.keys():
				setattr(twist.linear, key, self.position_control[key].get_output( ))

			for key in self.orientation_control.keys( ):
				setattr(twist.angular, key, self.orientation_control[key].get_output( ))

			
			# point_data = Vector3Stamped()
			# point_data.header.frame_id = '/nav'
			# point_data.vector = twist.linear 
			# twist.linear = self.tfListener.transformVector3("/drone_local", point_data).vector


			#aux_x = twist.linear.x * cos(self.yaw) + twist.linear.y * sin(self.yaw)
			#aux_y = - twist.linear.x * sin(self.yaw) + twist.linear.y * cos(self.yaw)
			#twist.linear.x = aux_x
			#twist.linear.y = aux_y
			self.publisher['command_velocity'].publish(twist)

	def RecieveState(self, data):
		self.state = data.key 

def main():    
	rospy.init_node('Controller', anonymous = True)
	ros_handler = ROS_Handler()
	

	rospy.spin()

    

if __name__ == "__main__": main()

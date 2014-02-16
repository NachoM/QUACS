#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

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
from math import pi

Command_Time = 0.01;
Step = dict( 
	x = 0.1, 
	y = 0.1, 
	z = 0.1, 
	yaw = pi/20)

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

		# self.quadrotor = Quadrotor()
		self.Ts = kwargs.get('Command_Time', Command_Time)
		self.name = kwargs.get('name', "/goal")
		self.signal = []


		self.publisher.update(
			land = rospy.Publisher('/ardrone/land', Empty, latch = True),
			takeoff = rospy.Publisher('/ardrone/takeoff', Empty, latch = True),
			reset = rospy.Publisher('/ardrone/reset', Empty, latch = True),
			controller_state = rospy.Publisher('/ardrone/controller/state', KeyValue, latch = True),
			cmd_vel = rospy.Publisher('/cmd_vel', Twist),
			trajectory = rospy.Publisher('/ardrone/trajectory', Odometry) )

		self.subscriber.update(
			ardrone_state = rospy.Subscriber('/ardrone/navdata', Navdata, callback = self.RecieveNavdata),
			sonar_height = rospy.Subscriber('/sonar_height', Range, callback = self.ReceiveSonarHeight )
			)

		self.services.update(
			signal_service = rospy.Service('/ardrone_control/Signal', Signal, self.Signal)
			)
		
		self.tf_broadcaster.update( 
			goal_tf = tf.TransformBroadcaster() 
			)

		self.timer.update(
			trajectory_timer = rospy.Timer(rospy.Duration( self.Ts ), self.Talk, oneshot=False),
			signal_timer = None
			)


		
		
	def RecieveNavdata( self, data ):
		self.set_state(data.state)

	def ReceiveSonarHeight(self, data):
		"""
		if data.max_range > data.range:
			self.landFlag = False
		if data.range < safe_min:
			pass
		"""
		pass

	def Signal(self, data):
		if len(self.signal):
			print "It's still sending data"
		else:
			if self.state == 'Flying' or self.state == 'Hovering':
				self.ControlOff()
				self.signal = SignalResponse( tf = data.time, dt = data.dt, f = data.f, signal = data.signal, direction = data.direction ) 
				self.timer['signal_timer'] = rospy.Timer( rospy.Duration(self.signal.dt), self.CmdSignal, oneshot = False )
		return []

	def CmdSignal(self, time):
		msg = Twist()
		if len(self.signal):
			if self.signal.direction == 'yaw':
				msg.angular.z = self.signal.command() #set angular velocity
			else:
				setattr( msg.linear, self.signal.direction, self.signal.command() ) #set linear velocity
		else:
			print "Done sending Signal"
			self.SignalOff()

		self.publisher['cmd_vel'].publish( msg ) #always publish so last command makes it stop

	def SignalOff(self):
		try:
			self.timer['signal_timer'].shutdown( )
			self.signal = [ ]
		except AttributeError:
			pass

		msg = Twist()
		self.publisher['cmd_vel'].publish( msg ) 

	def Talk( self, time ):
		""" Publishes the Goal Point for the Controller """

		msgs = Odometry( )
		msgs.header.stamp = rospy.Time.now()
		msgs.header.frame_id = "/nav"
		msgs.child_frame_id = "/goal" 

		# position
		msgs.pose.pose.position.x = self.position.x
		msgs.pose.pose.position.y = self.position.y
		msgs.pose.pose.position.z = self.position.z

		# orientation
		msgs.pose.pose.orientation.x = self.orientation.x
		msgs.pose.pose.orientation.y = self.orientation.y
		msgs.pose.pose.orientation.z = self.orientation.z
		msgs.pose.pose.orientation.w = self.orientation.w
		
		# linear velocity
		msgs.twist.twist.linear.x = self.velocity.x
		msgs.twist.twist.linear.y = self.velocity.y
		msgs.twist.twist.linear.z = self.velocity.z

		# angular velocity
		msgs.twist.twist.angular.x = self.velocity.roll
		msgs.twist.twist.angular.y = self.velocity.pitch
		msgs.twist.twist.angular.z = self.velocity.yaw

		self.publisher['trajectory'].publish(msgs)

		self.tf_broadcaster['goal_tf'].sendTransform( 
			(msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)

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
	
	def X(self, scale):
		self.change_set_point('x', scale)

	def Y(self, scale):
		self.change_set_point('y', scale)

	def Z(self, scale):
		self.change_set_point('z', scale)

	def Yaw(self, scale):
		self.change_set_point('yaw', scale) 
		# Order is important: first change set point in yaw, then update quaternions.
		if self.position.yaw >=  pi :
			self.position.yaw -= 2 * pi;
		if self.position.yaw < -pi:
			self.position.yaw += 2 * pi;

		quaternion = tf.transformations.quaternion_from_euler(self.position.yaw, self.position.pitch, self.position.roll, 'rzyx')
		
		self.orientation.x = quaternion[0]
		self.orientation.y = quaternion[1]
		self.orientation.z = quaternion[2]
		self.orientation.w = quaternion[3]

	def ControlOff(self, *args):
		self.change_controller_state('Off')

	def ControlOn(self, *args):
		if self.state == 'Flying' or self.state == 'Hovering':
			self.change_controller_state('On')

	def default_chirp_data(self, direction):
		data = Signal()
		data.time = 10
		data.dt = 0.01
		data.f = 20
		data.signal = 'chirp'
		data.direction = direction

		return data

	def ChirpX(self, *args):
		self.Signal( self.default_chirp_data('x') )

	def ChirpY(self, *args):
		self.Signal( self.default_chirp_data('y') ) 

	def ChirpZ(self, *args):
		self.Signal( self.default_chirp_data('z') ) 

	def ChirpYaw(self, *args):
		self.Signal( self.default_chirp_data('yaw') ) 

	def change_set_point(self, direction, scale):
		# it changes an attribute from the self.position by a fraction of a Step.
		setattr(self.position, direction, 
			getattr(self.position, direction) + scale * Step[direction] )

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
	#rospy.init_node('StateHandler', anonymous = True)
	ros_handler = ROS_Handler(Command_Time = 1)
	

	rospy.spin()
		


    

if __name__ == "__main__": main()
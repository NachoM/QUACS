#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range, NavSatFix
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# from Odometry import Odometry as AROdometry
from Quadrotor import Quadrotor 
import Process
from ROS import ROS_Object 

from math import pi

import tf

# Some Constants
Command_Time = 0.005;
g = 9.81;

class ROS_Odometry(Quadrotor, ROS_Object, object):
    """docstring for SensorFusion
    Subscribed to:
        ardrone/navdata -> reads ardrone raw navdata 
        fix -> reads gps 
        ardrone/imu -> reads imu for complementary filter 
        ardrone/sonar_height -> for Z proximity 

    Publishes to:
        /ardrone/sensorfusion/navdata -> Odometry estimated state of ardrone 
            It is called by a Timer
            Published as a tf too. 

    """
    
    def __init__(self, **kwargs ):
        super(ROS_Odometry, self).__init__(**kwargs)
        self.name = kwargs.get('name', "/local")

        self.subscriber = dict(
            raw_navdata = rospy.Subscriber('/ardrone/navdata',Navdata, callback = self.ReceiveNavdata ),
            raw_gps = rospy.Subscriber('/fix', NavSatFix, callback = self.ReceiveGPS),
            raw_imu = rospy.Subscriber('/ardrone/imu', Imu, callback = self.ReceiveImu ),
            raw_sonar = rospy.Subscriber('/sonar_height', Range, callback = self.ReceiveSonarHeight )
            )

        self.publisher = dict(
            state = rospy.Publisher('/ardrone/sensorfusion/navdata', Odometry)
            )
        
        self.tf_broadcaster = dict( 
            local_tf = tf.TransformBroadcaster()
            )

        self.timer = dict( 
            publish_timer = rospy.Timer(rospy.Duration( Command_Time ), self.Command, oneshot=False)
            )

    def Command(self, time):
        self.Talk()
        self.predict()

    def Talk(self):
        msgs = Odometry()
        msgs.header.stamp = rospy.Time.now()
        msgs.header.frame_id = "/nav"
        msgs.child_frame_id = self.name 


        msgs.pose.pose.position.x = self.position.x
        msgs.pose.pose.position.y = self.position.y
        msgs.pose.pose.position.z = self.position.z

        # quaternion = tf.transformations.quaternion_from_euler(self.position.yaw, self.position.pitch, self.position.roll, 'rzyx')
        
        msgs.pose.pose.orientation.x = self.orientation.x
        msgs.pose.pose.orientation.y = self.orientation.y
        msgs.pose.pose.orientation.z = self.orientation.z
        msgs.pose.pose.orientation.w = self.orientation.w 
        
        msgs.twist.twist.linear.x = self.velocity.x
        msgs.twist.twist.linear.y = self.velocity.y
        msgs.twist.twist.linear.z = self.velocity.z

        #rospy.loginfo(msgs)
        
        self.publisher['state'].publish(msgs)

        self.tf_broadcaster['local_tf'].sendTransform( (msgs.pose.pose.position.x, msgs.pose.pose.position.y, msgs.pose.pose.position.z) , 
            (msgs.pose.pose.orientation.x, msgs.pose.pose.orientation.y, msgs.pose.pose.orientation.z, msgs.pose.pose.orientation.w), 
            msgs.header.stamp, 
            msgs.child_frame_id, 
            msgs.header.frame_id)

    def ReceiveNavdata(self, navdata):
        self.position.set_properties(dict(roll = navdata.rotX * pi/180.0, pitch = navdata.rotY * pi/180.0, yaw = navdata.rotZ * pi/180.0))
        self.velocity.set_properties(dict(x = navdata.vx / 1000.0, y = navdata.vy / 1000.0, z = navdata.vz / 1000.0))
        self.acceleration.set_properties(dict(x = navdata.ax * g, y = navdata.ay * g, z = (navdata.az - 1.0) * g ))
        self.orientation.set_euler(roll = navdata.rotX * pi/180.0 , pitch = navdata.rotY * pi/180.0 , yaw = navdata.rotZ * pi/180.0  )
        
        self.set_state(navdata.state)
        self.battery = navdata.batteryPercent
        
    def ReceiveGPS(self, gpsdata):
        pass

    def ReceiveImu(self, ImuRaw):
        """ Receive RAW imu msgs""" 
        pass

    def ReceiveSonarHeight(self, Range):
        """ Receive Sonar Heigh proximity msgs"""
        pass


def main():
    rospy.init_node('SensorFusion_Odometry', anonymous = True)
    node = SensorFusion( processes = [Process.XY_Odometry1(Ts = Command_Time), Process.Z_Odometry1(Ts = Command_Time) ] )
    rospy.spin()

    

if __name__ == "__main__": main()
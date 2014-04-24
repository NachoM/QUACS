#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range, NavSatFix
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# from Odometry import Odometry as AROdometry
# from Quadrotor import Quadrotor 
from SensorFusion import SensorFusion 
import Sensors
import Process
from math import pi

import tf

# Some Constants
Command_Time = 0.005;
g = 9.81;

class ROS_SensorFusion(SensorFusion, object):
    """docstring for ROS_SensorFusion:
    
    Subscribed to:
        ardrone/navdata -> reads ardrone raw navdata 
        fix -> reads gps 

    Publishes to:
        /ardrone/sensorfusion/navdata -> Odometry estimated state of ardrone 
            It is called by a Timer
            Published as a tf too. 

    Executes a EKF with /fix and predicted position. 
    """

    def __init__(self, **kwargs):
        super(ROS_SensorFusion, self).__init__(**kwargs)

        # rospy.Subscriber('fix', GPS, callback = self.Listen, callback_args = self.ReceiveGPS)
        rospy.Subscriber('ardrone/navdata',Navdata, callback = self.ReceiveNavdata) 
        # rospy.Subscriber('ardrone/imu', Imu, callback = self.ReceiveImu)
        rospy.Subscriber('fix', NavSatFix, callback = self.ReceiveGPS)
        self.gps_calibrated = False
        # rospy.Subscriber('/sonar_height', Imu, callback = self.ReceiveSonarHeight)

        self.publisher = rospy.Publisher('ardrone/sensorfusion/navdata', Odometry)

        rospy.Timer(rospy.Duration( Command_Time ), self.FuseSensors, oneshot=False)

    def FuseSensors(self, arg):
        # self.predict()
        
        self.Talk() #publishes data

    def Talk(self):
        """ Publishes in std msg form the estimated state """
        msgs = Odometry()
        msgs.header.stamp = rospy.Time.now()
        msgs.header.frame_id = "local"


        msgs.pose.pose.position.x = self.position.x
        msgs.pose.pose.position.y = self.position.y
        msgs.pose.pose.position.z = self.position.z

        quaternion = tf.transformations.quaternion_from_euler(self.odometry.position.yaw, self.odometry.position.pitch, self.odometry.position.roll, 'rzyx')
        msgs.pose.pose.orientation.x = quaternion[0]
        msgs.pose.pose.orientation.y = quaternion[1]
        msgs.pose.pose.orientation.z = quaternion[2]
        msgs.pose.pose.orientation.w = quaternion[3]

        """msgs.pose.pose.orientation.x = self.orientation.x 
        msgs.pose.pose.orientation.y = self.orientation.y
        msgs.pose.pose.orientation.z = self.orientation.z
        msgs.pose.pose.orientation.w = self.orientation.w
        """

        
        msgs.twist.twist.linear.x = self.velocity.x
        msgs.twist.twist.linear.y = self.velocity.y
        msgs.twist.twist.linear.z = self.velocity.z

        #rospy.loginfo(msgs)
        
        self.publisher.publish(msgs)

    def ReceiveNavdata(self, navdata):
        self.position = dict(roll = navdata.rotX * 180.0 / pi, pitch = navdata.rotY * 180.0 / pi, yaw = navdata.rotZ * 180.0 / pi );
        self.velocity = dict(x = navdata.vx / 1000.0, y = navdata.vy / 1000.0, z = navdata.vz / 1000.0)
        self.acceleration = dict(x = navdata.ax * g, y = navdata.ay * g, z = navdata.az * g)

        self.state = navdata.state
        self.battery = navdata.batteryPercent

        # self.update()

        # self.Talk()

    def ReceiveGPS(self, gpsdata):
        """ Receive GPS sensor msgs NavSatFix type
        measure -> get Z vector 
        map -> get residual vector 
        """ 


        for sensor in self.sensors:
            if sensor == 'GPS':
                if not self.gps_calibrated:
                    self.gps_calibrated = True
                    sensor.set_zero( latitude = gpsdata.latitude * pi/180.0, altitude = gpsdata.altitude, longitude = gpsdata.longitude * pi/180.0 )

                cartesian = gps2cart(latitude = gpsdata.latitude * pi/180.0 - sensor.gps_zero['latitude'], altitude = gpsdata.altitude - sensor.gps_zero['altitude'], 
                    longitude = gpsdata.longitude * pi/180.0 - sensor.gps_zero['longitude'])

                self.position = dict(x = cartesian['x'], y = cartesian['y'], z = cartesian['z'] );
                # sensor.measure(latitude = gpsdata.latitude * pi/180.0, altitude = gpsdata.altitude, longitude = gpsdata.longitude * pi/180.0)
                
                # sensor.map(x = self.position.x, y = self.position.y, z = self.position.z)
                

        # self.correct()

    def ReceiveImu(self, ImuRaw):
        """ Receive RAW imu msgs""" 
        pass

    def ReceiveSonarHeight(self, Range):
        """ Receive Sonar Heigh proximity msgs"""
        pass

    def set_measurement(self, sensor):
        i = 0
        for item in sensor.get_measurementList():
            self.Z[ self._measurementList.index( item )][0] = sensor.Z[i][0]
            i += 1

def main():
    rospy.init_node('SensorFusion_Odometry', anonymous = True)
    node = ROS_SensorFusion( sensors = [Sensors.GPS(), Sensors.DummyYaw()], processes = [Process.XY_Odometry1( Ts = Command_Time ) , Process.Z_Odometry1( Ts = Command_Time )] )
    rospy.spin()

    

if __name__ == "__main__": main()

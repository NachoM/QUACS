#!/usr/bin/env python

# Import the ROS libraries, and load the manifest file which through <depend package=.. /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_control')
import rospy

# Import the messages we're interested in sending and receiving
from sensor_msgs.msg import Imu, Range, NavSatFix
from nav_msgs.msg import Odometry 
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# from Odometry import Odometry as AROdometry
import Process
import Sensors
import SensorFusion

from Filter import DigitalFilter
from ROS_Odometry import ROS_Odometry 
from ROS_SensorFusion import ROS_SensorFusion


from math import pi

import tf

# Some Constants
Command_Time = 0.005;
IMU_Period = 0.01;
g = 9.81;

class FilteredOdometry(ROS_SensorFusion, object):
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

    low_pass_filters dictionary from navdata entries to corresponding butterworth filter 

    """
    def __init__(self, **kwargs ):
        super(FilteredOdometry, self).__init__(**kwargs)
        self.low_pass_filters = dict()
        filter_params = rospy.get_param( 'Navdata', dict() ) 

        for key, values in filter_params.items():
            self.low_pass_filters[key] = DigitalFilter( a = values['a'], b = values['b'] )

    @property 
    def low_pass_filters(self):
        return self.properties.get('low_pass_filters', None)
    @low_pass_filters.setter 
    def low_pass_filters(self, low_pass_filters):
        self.properties['low_pass_filters'] = low_pass_filters
    @low_pass_filters.deleter
    def low_pass_filters(self):
        del self.properties['low_pass_filters']
        
    def ReceiveNavdata(self, navdata):
        filtered_navdata = self.FilterNavdata(navdata)
        super(FilteredOdometry, self).ReceiveNavdata(filtered_navdata)

    def FilterNavdata(self, navdata):
        for key in self.low_pass_filters.keys():
            self.low_pass_filters[key].set_input( getattr(navdata, key) ) #input new data in filter
            setattr(navdata, key, self.low_pass_filters[key].get_output() ) #set in navdata filter's output

        return navdata

def main():
    rospy.init_node('SensorFusion', anonymous = True)

    node = FilteredOdometry( 
        sensors = dict( 
            imu = SensorFusion.IMU_Mahoney(  ), 
            #imu = SensorFusion.IMU_Magdwick(  ),
            gps = SensorFusion.GPS_Filter(  ) 
            ) 
        )
    
    rospy.spin()

    

if __name__ == "__main__": main()

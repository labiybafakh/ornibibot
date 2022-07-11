import sys
from typing_extensions import Self

import numpy as np
import matplotlib.pyplot as graph
from ypstruct import structure

import rospy
from geometry_msgs.msg import WrenchStamped


class PlotSensor:
    def __init__(self) -> None:
        force = np.zeros(shape=3, dtype=np.float32)
        moment= np.zeros(shape=3, dtype=np.float32)

        # self.force = structure()
        # self.force.x = np.zeros(dtype=np.float32)
        # self.force.y = np.zeros(dtype=np.float32)
        # self.force.z = np.zeros(dtype=np.float32)        

        # self.moment = structure()
        # self.moment.x = np.zeros(dtype=np.float32)
        # self.moment.y = np.zeros(dtype=np.float32)
        # self.moment.z = np.zeros(dtype=np.float32)


        self.sub_force      = rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.callback_force_sensor)
    
    def callback_force_sensor(self, force_data):
        force = np.append(  force_data.wrench.force.x,
                            force_data.wrench.force.y,
                            force_data.wrench.force.z )

class OrnibiBot:
    def __init__(self):
        self.initNode       = rospy.init_node('OrnibiBotPC', anonymous=False, log_level=rospy.DEBUG)
        self.rate           = rospy.Rate(100)
        
        self.current_time   = rospy.Time.now()
        self.last_time      = rospy.Time.now()
        self.deltaTime      = rospy.Time.now()


if __name__ == '__main__':

    sensor = PlotSensor()
    ornibibot = OrnibiBot()

    try:
        while not rospy.is_shutdown():
            rospy.rospy.loginfo("Force:{}".format(sensor.force))
            ornibibot.rate.sleep()
                    
    
    except rospy.ROSInterruptException : pass
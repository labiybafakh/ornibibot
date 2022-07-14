import sys
from typing_extensions import Self
from PyQt5 import QtCore, QtWidgets


import numpy as np
import matplotlib.pyplot as graph
from ypstruct import structure

import rospy
from geometry_msgs.msg import WrenchStamped


class PlotSensor:
    def __init__(self) -> None:
        self.force_x = []
        self.force_y = []
        self.force_z = []
        self.time    = []
        # self.force = np.zeros(3, dtype=np.float32)
        # self.moment= np.zeros(3, dtype=np.float32)
        # self.force = structure()
        # self.force.x = []
        # self.force.y = []
        # self.force.z = []        

        # self.moment = structure()
        # self.moment.x = []
        # self.moment.y = []
        # self.moment.z = []

        self.sub_force      = rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.callback_force_sensor)
    
    def callback_force_sensor(self, force_data):
        self.force_x.append(force_data.wrench.force.x)
        self.force_y.append(force_data.wrench.force.y)
        self.force_z.append(force_data.wrench.force.z)
        # self.force = np.append(self.force,  [force_data.wrench.force.x,
        #                     force_data.wrench.force.y,
        #                     force_data.wrench.force.z] )
        

class OrnibiBot:
    def __init__(self):
        self.initNode       = rospy.init_node('OrnibiBotPC', anonymous=False, log_level=rospy.DEBUG)
        self.rate           = rospy.Rate(100)
        
        self.current_time   = rospy.Time.now()
        self.last_time      = rospy.Time.now()
        self.deltaTime      = rospy.Time.now()

if __name__ == '__main__':
    
    ornibibot = OrnibiBot()
    sensor = PlotSensor()

    try:
        while not rospy.is_shutdown():
            # rospy.loginfo("Force:{}".format(sensor.force))
            ornibibot.rate.sleep()
            # ornibibot.time = np.append(ornibibot.time, [rospy.Time.now().to_sec(),rospy.Time.now().to_sec(),rospy.Time.now().to_sec()])
        
        graph.plot(sensor.time, sensor.force_x)
        graph.plot(sensor.time, sensor.force_y)
        graph.plot(sensor.time, sensor.force_z)
        graph.xlabel('Time')
        graph.ylabel('Force')
        graph.title('Static Force')
        graph.legend()
        graph.tight_layout()
        graph.show()


    except rospy.ROSInterruptException : pass
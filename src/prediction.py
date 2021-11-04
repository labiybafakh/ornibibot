#!/usr/bin/env python

import rospy
from std_msgs.msg   import *
from nav_msgs.msg   import *
from geometry_msgs  import * 
from vectornav.msg  import dThetaVel, trueBody

# rpy = [], acc = [], gyro = [] #trueBody
# dt, dVel=[] #Delta Theta Velocity


def callbackTrueBody(trueBodyData):
    global a
    rpy=[]

    a = trueBodyData.RPY.x

def callbackdThetaVel(dThetaVelData):
    rospy.loginfo(a)

    

def main():    
    rospy.init_node('OrnibiBotPC', anonymous=False)
    sub_trueBody    = rospy.Subscriber('vectornav/trueBody', trueBody, callbackTrueBody )
    sub_dThetaVel   = rospy.Subscriber('vectornav/dThetaVel', dThetaVel, callbackdThetaVel )


    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass


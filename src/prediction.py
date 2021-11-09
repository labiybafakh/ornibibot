#!/usr/bin/env python

import sys

from numpy.lib.financial import rate
import rospy
# from std_msgs.msg import *
# from geometry_msgs.msg import *
from vectornav.msg  import dThetaVel, trueBody
import time
import tf2_py
# import tf2_ros
# import tf2_msgs.msg
import tf_conversions
from nav_msgs.msg import *
from geometry_msgs.msg import *
import geometry_msgs.msg
import std_msgs.msg
# import nav_msgs.msg


class PositionEstimation:

    def __init__(self):

        self.acc    = [0, 0, 0]
        self.rpy    = [0, 0, 0]
        self.angular= [0, 0, 0]
        self.dVel   = [0, 0, 0]
        self.dt     = 0
        self.posXYZ = [0, 0, 0]


        self.sub_trueBody   = rospy.Subscriber('vectornav/trueBody', trueBody, self.callbackTrueBody )
        self.sub_dThetaVel  = rospy.Subscriber('vectornav/dThetaVel', dThetaVel, self.callbackdThetaVel )
        self.odom_pub       = rospy.Publisher("odom", Odometry, queue_size=50)

        self.initNode       = rospy.init_node('OrnibiBotPC', anonymous=False)


        self.rate           = rospy.Rate(50)
        
        self.current_time   = rospy.Time.now()
        self.last_time      = rospy.Time.now()
        self.deltaTime      = rospy.Time.now()


    def callbackTrueBody(self, trueBodyData):

        # print(trueBodyData)

        self.rpy[0]     = trueBodyData.RPY.x
        self.rpy[1]     = trueBodyData.RPY.y
        self.rpy[2]     = trueBodyData.RPY.z

        self.acc[0]     = trueBodyData.bodyAcc.x
        self.acc[1]     = trueBodyData.bodyAcc.y
        self.acc[2]     = trueBodyData.bodyAcc.z

        self.angular[0] = trueBodyData.gyro.x
        self.angular[1] = trueBodyData.gyro.y
        self.angular[2] = trueBodyData.gyro.z

        # self.publish(trueBodyData)
                

    def callbackdThetaVel(self, dThetaVelData):

        self.dt      = dThetaVelData.deltaTime

        self.dVel[0]    = dThetaVelData.deltaVelocity.x
        self.dVel[1]    = dThetaVelData.deltaVelocity.y
        self.dVel[2]    = dThetaVelData.deltaVelocity.z        

    # def tf_imu(self):

    #     odom_broadcaster    = tf2_py.
    #     tf2_ros.TransformBroadcaster()
    #     t                   = geometry_msgs.msg.TransformStamped()
    #     self.current_time   = rospy.Time.now()
        
    #     self.deltaTime      = (self.current_time - self.last_time).to_sec()

    #     self.posXYZ[0]      += self.dVel[0]*self.dt
    #     self.posXYZ[1]      += self.dVel[1]*self.dt
    #     self.posXYZ[2]      += self.dVel[2]*self.dt

    #     t.header.stamp      = self.current_time
    #     t.header.frame_id   = "map"
    #     t.child_frame_id    = "base_link"

    #     t.transform.translation.x   = self.acc[0]
    #     t.transform.translation.y   = self.acc[1]
    #     t.transform.translation.z   = self.acc[2]

    #     odom_quat   = tf_conversions.transformations.quaternion_from_euler(
    #                 self.rpy[0],self.rpy[1],self.rpy[2])

    #     t.transform.rotation.x  = odom_quat[0]
    #     t.transform.rotation.y  = odom_quat[1]
    #     t.transform.rotation.z  = odom_quat[2]
    #     t.transform.rotation.w  = odom_quat[3]

    #     odom_broadcaster.sendTransform(t)

    #     odom                = Odometry()
    #     odom.header.stamp   = self.current_time
    #     odom.header.frame_id= "odom"

    #     odom.pose.pose      = Pose(Point(self.posXYZ[0],self.posXYZ[1],self.posXYZ[2]), Quaternion(*odom_quat))

    #     odom.child_frame_id = "base_link"
    #     odom.twist.twist    = Twist(Vector3(self.dVel[0],self.dVel[2],self.dVel[2]),
    #                     Vector3(self.angular[0], self.angular[1], self.angular[2]))

    #     self.odom_pub.publish(odom)

    #     self.last_time   = self.current_time


    
if __name__ == '__main__':

    posEst = PositionEstimation()

    try:
        while not rospy.is_shutdown():
            # posEst.print()
            posEst.tf_imu()
            #print "Acc:{}", posEst.acc
            #posEst.rate.sleep()
            rospy.spin()
            
    
    except rospy.ROSInterruptException : pass

#!/usr/bin/env python
import roslib
roslib.load_manifest('my_turtle_controller'); import rospy

from turtlesim.msg import *

vel = Velocity()
vel.linear = 1.
vel.angular = 1.

pub = rospy.Publisher('/turtle1/command_velocity', Velocity)
rospy.init_node('controller')

while not rospy.is_shutdown():
    pub.publish(vel)
    rospy.sleep(1.0)

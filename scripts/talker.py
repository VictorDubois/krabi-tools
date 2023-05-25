#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from time import sleep
from std_msgs.msg import String, Bool
from krabi_msgs.msg import odom_lighter, motors_cmd, motors_parameters
from geometry_msgs.msg import Twist

class send_to_motors:
    def __init__(self) -> None:
        pass

    def odom_lighter_callback(self, data) -> None:
        print("coucou")
    
    def setup(self)-> None:
        rospy.Subscriber("odom_lighter", odom_lighter, self.odom_lighter_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.motors_cmd_pub = rospy.Publisher('motors_cmd', motors_cmd, queue_size=10)
        self.enable_motors_pub = rospy.Publisher('enable_motors', Bool, queue_size=10)
        self.motors_parameters_pub = rospy.Publisher('motors_parameters', motors_parameters, queue_size=10)
        rospy.init_node('odom_calib_tool', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.angular.x = 0
        self.cmd_vel_msg.angular.y = 0
        self.cmd_vel_msg.angular.z = 0
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.linear.z = 0

        self.enable_motors_msg = Bool()
        self.enable_motors_msg.data = True

        self.motors_parameters_msg = motors_parameters()
        self.motors_parameters_msg.max_current = 0.7
        self.motors_parameters_msg.max_current_left = 0.7
        self.motors_parameters_msg.max_current_right = 0.7

        self.motors_cmd_msg = motors_cmd()
        self.motors_cmd_msg.enable_motors = True
        self.motors_cmd_msg.override_PWM = False
        self.motors_cmd_msg.PWM_override_left = 0
        self.motors_cmd_msg.PWM_override_right = 0
        self.motors_cmd_msg.reset_encoders = True
        self.publish_all()

        sleep(2)
        self.motors_cmd_msg.reset_encoders = False

        while not rospy.is_shutdown():
            self.publish_all()
            rate.sleep()

    def publish_all(self) -> None:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            self.motors_parameters_pub.publish(self.motors_parameters_msg)
            self.enable_motors_pub.publish(self.enable_motors_msg)
            self.motors_cmd_pub.publish(self.motors_cmd_msg)
        

if __name__ == '__main__':
    try:
        my_instance = odom_calibrator()
        my_instance.setup()
    except rospy.ROSInterruptException:
        pass

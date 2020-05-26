#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

#scale factors
accelScale = 0.75
joyScale = 1.0

right_trigger = 0.0
left_trigger = 0.0
left_stick_lr = 0.0


def joyCallback(joyMessage):
    print('Input Received')
    
    #uncomment the below lines to find axes and button mappings for your controller. I am using an xbox controller
    #print(joyMessage.axes)
    #print(joyMessage.buttons)

    #normalise [-1,1] inputs to [0,1]
    right_trigger = (1 - joyMessage.axes[5])/2
    left_trigger = -(1-joyMessage.axes[2])/2
    left_stick_lr = joyMessage.axes[0]

    publishVelocity(right_trigger,left_trigger,left_stick_lr)

def publishVelocity(forwardAccel,revAccel, turnLR):
    velocity_message = Twist()
    velocity_message.linear.x = (forwardAccel + revAccel) * accelScale
    velocity_message.angular.z = turnLR * joyScale 
    velocity_pub.publish(velocity_message)
    print('Published Velocity: '+str(velocity_message.linear.x)+' Turn: '+str(velocity_message.angular.z))
   
if __name__ == '__main__':
    rospy.init_node('joy_teleop',anonymous=True)
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.spin()
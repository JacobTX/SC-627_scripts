#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi,sqrt


bot_pose1=[0,0,0]
bot_pose2=[0,0,0]
bot_pose3=[0,0,0]
bot_pose4=[0,0,0]

mode = 1 # Balanced or Synchronized

m = 0

c = 0
s = 0
U = 100

def sgn(x):
    # returns the sign of x
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

def distance(A,B):
    # returns the distance between two points
    return ((A[0]-B[0])**2 + (A[1]-B[1])**2)**0.5

def quat_to_yaw(w,z):
    if (z >=0):
        return (2*acos(w))
    elif (z < 0):
        return (2*(pi - acos(w)))


def odomdata_callback1(data):
    # obtains the pose of the bot
    xloc1 = data.pose.pose.position.x
    yloc1 = data.pose.pose.position.y
    z1 = data.pose.pose.orientation.z
    w1 = data.pose.pose.orientation.w
    theta1 = quat_to_yaw(w1,z1)

    
    global bot_pose1, bot_pose2, bot_pose3, bot_pose4, c, s, U, v, k
    bot_pose1 = [xloc1,yloc1,theta1] 

    c = cos(bot_pose1[2])+cos(bot_pose2[2])+cos(bot_pose3[2])+cos(bot_pose4[2])
    s = sin(bot_pose1[2])+sin(bot_pose2[2])+sin(bot_pose3[2])+sin(bot_pose4[2])
    U = 0.5*(c**2+s**2)

    print(U)
    print(bot_pose1[2],bot_pose2[2],bot_pose3[2],bot_pose4[2])

    move_the_bot1 = Twist()
    move_the_bot1.linear.x = v
    move_the_bot1.angular.z = w0 + k*(-c*sin(bot_pose1[2])+s*cos(bot_pose1[2]))
    
    publish_to_cmd_vel1.publish(move_the_bot1) 

def odomdata_callback2(data):
        # obtains the pose of the bot
    xloc2 = data.pose.pose.position.x
    yloc2 = data.pose.pose.position.y
    z2 = data.pose.pose.orientation.z
    w2 = data.pose.pose.orientation.w
    theta2 = quat_to_yaw(w2,z2)

    global bot_pose1, bot_pose2, bot_pose3, bot_pose4, c, s, v, k
    bot_pose2 = [xloc2,yloc2,theta2]
    
    move_the_bot2 = Twist()
    move_the_bot2.linear.x = v
    move_the_bot2.angular.z = w0 + k*(-c*sin(bot_pose2[2])+s*cos(bot_pose2[2]))
   
    publish_to_cmd_vel2.publish(move_the_bot2)  

def odomdata_callback3(data):
    # obtains the pose of the bot
    xloc3 = data.pose.pose.position.x
    yloc3 = data.pose.pose.position.y
    z3 = data.pose.pose.orientation.z
    w3 = data.pose.pose.orientation.w
    theta3 = quat_to_yaw(w3,z3)

    global bot_pose1, bot_pose2, bot_pose3, bot_pose4, c, s, v, k
    bot_pose3 = [xloc3,yloc3,theta3] 
    
    move_the_bot3 = Twist()
    move_the_bot3.linear.x = v
    move_the_bot3.angular.z = w0 + k*(-c*sin(bot_pose3[2])+s*cos(bot_pose3[2]))
    
    publish_to_cmd_vel3.publish(move_the_bot3) 

def odomdata_callback4(data):
        # obtains the pose of the bot
    xloc4 = data.pose.pose.position.x
    yloc4 = data.pose.pose.position.y
    z4 = data.pose.pose.orientation.z
    w4 = data.pose.pose.orientation.w
    theta4 = quat_to_yaw(w4,z4)

    global bot_pose1, bot_pose2, bot_pose3, bot_pose4, c, s, v, k
    bot_pose4 = [xloc4,yloc4,theta4] 
    

    move_the_bot4 = Twist()
    move_the_bot4.linear.x = v
    move_the_bot4.angular.z = w0 + k*(-c*sin(bot_pose4[2])+s*cos(bot_pose4[2]))
   
    publish_to_cmd_vel4.publish(move_the_bot4)  
    
if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    rospy.loginfo('My node has been started')

    if (mode == 0):
        # Balanced
        k = -5  
        v = 0.1 
        w0 = 0.1  
    elif (mode == 1):
        # Sync
        k = 1   
        v = 0.1
        w0 = 0
       
    publish_to_cmd_vel1 = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
    publish_to_cmd_vel2 = rospy.Publisher('/bot_2/cmd_vel', Twist, queue_size = 10)
    publish_to_cmd_vel3 = rospy.Publisher('/bot_3/cmd_vel', Twist, queue_size = 10)
    publish_to_cmd_vel4 = rospy.Publisher('/bot_4/cmd_vel', Twist, queue_size = 10)
    

    #subscribe to odometry data to get the bot's pose
    subscribe_to_odom1 = rospy.Subscriber('/bot_1/odom',Odometry, callback = odomdata_callback1) 
    subscribe_to_odom2 = rospy.Subscriber('/bot_2/odom',Odometry, callback = odomdata_callback2) 
    subscribe_to_odom3 = rospy.Subscriber('/bot_3/odom',Odometry, callback = odomdata_callback3) 
    subscribe_to_odom4 = rospy.Subscriber('/bot_4/odom',Odometry, callback = odomdata_callback4) 

    rospy.spin()


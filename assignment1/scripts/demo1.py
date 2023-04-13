#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi


bot_pose = [] # Current pose of the bot
goal_loc = [2,2] #Goal location
min_dist = 10 #minimum distance between bot and goal
min_loc = [] #location from which distance to goal is minimum
start_loc = [] #location from which circumnavigation starts
direc = [] #vector from bot to goal location

d_crit = 0.25 # critical distance to avoid collision
d_reached = 0.01 # critical distance to conclude that bot has reached the goal location
w0 = 0.5  # characteristic value of angular velocity
v0 = 0.1  # characteristic value of translational velocity
s = 1 # switch variable to transition from 'moving towards goal' to 'circumnavigation'
c = 0 # counter to track whether circumnavigation started
n = 0 # counter to track whether first loop has been completed

d0 = 0
d45 = 0
d135 = 0

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

def odomdata_callback(data):
    # obtains the pose of the bot
    xloc = data.pose.pose.position.x
    yloc = data.pose.pose.position.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w


    if (z >=0):
        theta = 2*acos(w)
    elif (z < 0):
        theta = 2*(pi - acos(w))

    global bot_pose
    bot_pose = [xloc,yloc,theta] 

def laserdata_callback(msg):

    
    global s, min_dist, min_loc, start_loc, d_crit, d_reached, direc, w0, v0, c, n
    global d0, d45, d135

    # receives the data from the laser scanner
    L=[1]*360
    
    for i in range(len(msg.ranges)):
        if msg.ranges[i] != 0:
            L[i]=msg.ranges[i]

    d0 = L[120]
    d45 = L[135]
    d135 = L[210]
    #print(d0,d45, d135, distance(bot_pose,goal_loc))

    
    # Bug 1 algorithm

    if distance(bot_pose,goal_loc) < d_reached:
        # when bot reaches the goal location
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0
        print("Reached the goal location", direc, d0)
    
    elif (distance(bot_pose,goal_loc) > d_reached) and (d0 > d_crit) and (s==1):
        # moving towards goal
        direc = [goal_loc[0]-bot_pose[0], goal_loc[1]-bot_pose[1]]
        orient = [cos(bot_pose[2]),sin(bot_pose[2])]

        v = sgn(direc[0]*orient[1]-direc[1]*orient[0])
        t = (direc[0]*orient[0]+direc[1]*orient[1])/distance(goal_loc,bot_pose)

        if v >= 0:
            p = acos(t)
        else:
            p = -acos(t)

        print("Moving towards goal", direc, d0)
        move_the_bot.linear.x = 0.05*(1+sgn((0.01-p)*(0.01+p)))
        move_the_bot.angular.z = -0.1*v
        start_loc = bot_pose
     
    else:
        # circumnavigate
        s = 0
        print("Circumnavigating", n, c, min_dist, min_loc, direc, d0)
        if (c==0):
            w = w0 * sgn(d45-d135) #corrective angular velocity
            dx = min(d0,d45,d135)
            v = 0.5 * v0 * (1 + sgn(dx-d_crit)) #corrective translation velocity
            #v=0
                
            move_the_bot.linear.x = v
            move_the_bot.angular.z = w

            
            # Calculation of minimum distance between bot and goal
            dist =  distance(goal_loc,bot_pose)
            dist2 = distance(start_loc,bot_pose)
            
            if dist < min_dist:
                min_dist = dist
                min_loc = bot_pose
            
            if dist2 > 0.2:
                n = 1 #First loop started
            
            if dist2 < 0.1 and (n==1):
                c = 1 #First loop ended
            

        elif (c==1):
            w = w0 * sgn(d45-d135) 
            dx = min(d0,d45,d135)
            v = 0.5 * v0 * (1 + sgn(dx-d_crit)) 
                
            move_the_bot.linear.x = v
            move_the_bot.angular.z = w

            if distance(min_loc,bot_pose)<0.1:
                move_the_bot.linear.x = 0
                move_the_bot.angular.z = 0
                s = 1
                n = 0
                c = 0
        
    
    publish_to_cmd_vel.publish(move_the_bot) 

if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/tb3_4/scan', LaserScan, callback = laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size = 10)

    #subscribe to odometry data to get the bot's pose
    subscribe_to_odom = rospy.Subscriber('/tb3_4/odom',Odometry, callback = odomdata_callback) 

    #create an object of Twist data

    move_the_bot = Twist()

    rospy.spin()


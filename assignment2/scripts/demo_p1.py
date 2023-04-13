#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from math import cos, sin, acos, pi,sqrt



bot_pose = [] # Current pose of the bot
goal_loc = [1,-2] #Goal location
prev_pose = [100,100,0]

#obst1_loc = [0.435507,-0.4709353]
#obst2_loc = [-1.2466,-0.865677]
#obst3_loc = [0.3216,1.3261021167]
#obst4_loc = [-1.1072,0.898614]

obst1_loc = [0.5,-1.1]


min_dist = 10 #minimum distance between bot and goal
min_loc = [] #location from which distance to goal is minimum
start_loc = [] #location from which circumnavigation starts
direc = [] #vector from bot to goal location

zeta = 0.1
eta1 = 0.03


d0_star = 0.05 # critical distance to conclude that bot has reached the goal location
Q1_star = 0.5
r1 = 0.1

bot_x = []
bot_y = []

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

def vicondata_callback(data):
    global obst1_loc

    obst1_loc = [data.transform.translation.x, data.transform.translation.y]

    print(obst1_loc)


def odomdata_callback(data):
    # obtains the pose of the bot
    xloc = data.pose.pose.position.x
    yloc = data.pose.pose.position.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w

    
    # convert quaternion to yaw
    if (z >=0):
        theta = 2*acos(w)
    elif (z < 0):
        theta = 2*(pi - acos(w))

    global bot_pose
    bot_pose = [xloc,yloc,theta] 
    bot_x.append(xloc)
    bot_y.append(yloc)

def bot_loc_plot(A,B):

    figure,axes = plt.subplots()
    
    axes.add_artist(plt.Circle(obst1_loc,0.1*r1,fill=False))
    axes.plot(A,B,'b')
    axes.plot([2,-2,-2,2,2],[2,2,-2,-2,2],'r')
    axes.set_xlabel('Bot x-coordinate')
    axes.set_ylabel('Bot y-coordinate')
    axes.set_title('Plot of Bot location')
    plt.show()

def laserdata_callback(msg):

    global prev_pose, bot_pose, direc, goal_loc, obst1_loc, obst2_loc, obst3_loc, obst4_loc

    d0 = distance(bot_pose,goal_loc)
    d1 = distance(bot_pose,obst1_loc)
    
    expr0 = (d0 - d0_star)
    expr1 = (d1 - (r1+Q1_star))
   

    print((expr0 < 0),(expr1 < 0))

    if (expr0 < 0):
        vec0 = [zeta*(goal_loc[0]-bot_pose[0]), zeta*(goal_loc[1]-bot_pose[1])]
    else:
        vec0 = [(zeta/d0)*(goal_loc[0]-bot_pose[0]), (zeta/d0)*(goal_loc[1]-bot_pose[1])]

    if (expr1 < 0):
        vec1 = [eta1*(bot_pose[0]-obst1_loc[0])/(d1**2),eta1*(bot_pose[1]-obst1_loc[1])/(d1**2)]
    else: 
        vec1 = [0,0]


    print(vec0,vec1)

    vec = [vec0[0]+vec1[0], vec0[1]+vec1[1]]
    mag = sqrt(vec[0]**2 + vec[1]**2)

    print(vec,mag)

    print("Distance to goal",distance(bot_pose,goal_loc))

    if distance(bot_pose,goal_loc ) < 0.05:
        print("Reached goal location")
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0
        bot_loc_plot(bot_x,bot_y)
        print(bot_x)
        print(bot_y)

    elif (distance(bot_pose,prev_pose) > 0.05):
        print("Orient")
        direc = [vec[0]/mag, vec[1]/mag]
        orient = [cos(bot_pose[2]),sin(bot_pose[2])]
        v = sgn(-direc[0]*orient[1] + direc[1]*orient[0])
        t = (direc[0]*orient[0]+direc[1]*orient[1])
        if v >= 0:
            p = acos(t)
        else:
            p = -acos(t)

        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0.1*v

        if (sgn(0.0001 - p**2) > 0):
            prev_pose = bot_pose
    else:
        print("Move")
        
        move_the_bot.linear.x = mag
        move_the_bot.angular.z = 0
    
    publish_to_cmd_vel.publish(move_the_bot) 


if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/tb3_1/scan', LaserScan, callback = laserdata_callback)
    #subscribe_to_vicon = rospy.Subscriber('/vicon/obstacle1/obstacle1',TransformStamped, callback = vicondata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size = 10)
    

    #subscribe to odometry data to get the bot's pose
    subscribe_to_odom = rospy.Subscriber('/tb3_1/odom',Odometry, callback = odomdata_callback) 

    #create an object of Twist data

    move_the_bot = Twist()

    rospy.spin()


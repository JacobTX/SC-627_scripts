#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi,sqrt
import matplotlib.pyplot as plt



bot_pose = [] # Current pose of the bot
goal_loc = [-0.5,-2.5] #Goal location
prev_pose = [100,100,0]

obst1_loc = [0,2]
obst2_loc = [1.5,0]
obst3_loc = [0,-2]
obst4_loc = [-1.5,0]

min_dist = 10 #minimum distance between bot and goal
min_loc = [] #location from which distance to goal is minimum
start_loc = [] #location from which circumnavigation starts
direc = [] #vector from bot to goal location

bot_x=[]
bot_y=[]


zeta = 0.05
eta1 = 0.03
eta2 = 0.03
eta3 = 0.03
eta4 = 0.03


d0_star = 0.05 # critical distance to conclude that bot has reached the goal location
Q1_star = 0.2
Q2_star = 0.2
Q3_star = 0.2
Q4_star = 0.2
r1 = 0.5
r2 = 0.7
r3 = 0.5
r4 = 0.5

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

def bot_loc_plot(A,B):

    figure,axes = plt.subplots()
    
    axes.add_artist(plt.Circle(obst1_loc,r1,fill=False))
    axes.add_artist(plt.Circle(obst2_loc,r2,fill=False))
    axes.add_artist(plt.Circle(obst3_loc,r3,fill=False))
    axes.add_artist(plt.Circle(obst4_loc,r4,fill=False))
    axes.plot(A,B,'b')
    axes.plot([3,-3,-3,3,3],[6,6,-6,-6,6],'r')
    axes.set_xlabel('Bot x-coordinate')
    axes.set_ylabel('Bot y-coordinate')
    axes.set_title('Plot of Bot location')
    plt.show()

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

def laserdata_callback(msg):

    global prev_pose, bot_pose, direc, goal_loc, obst1_loc, obst2_loc, obst3_loc, obst4_loc

    d0 = distance(bot_pose,goal_loc)
    d1 = distance(bot_pose,obst1_loc)
    d2 = distance(bot_pose,obst2_loc)
    d3 = distance(bot_pose,obst3_loc)
    d4 = distance(bot_pose,obst4_loc)

    
    expr0 = (d0 - d0_star)
    expr1 = (d1 - (r1+Q1_star))
    expr2 = (d2 - (r2+Q2_star))
    expr3 = (d3 - (r3+Q3_star))
    expr4 = (d4 - (r4+Q4_star))

    print((expr0 < 0),(expr1 < 0),(expr2 < 0),(expr3 < 0),(expr4 < 0))

    if (expr0 < 0):
        vec0 = [zeta*(goal_loc[0]-bot_pose[0]), zeta*(goal_loc[1]-bot_pose[1])]
    else:
        vec0 = [(zeta/d0)*(goal_loc[0]-bot_pose[0]), (zeta/d0)*(goal_loc[1]-bot_pose[1])]

    if (expr1 < 0):
        vec1 = [eta1*(bot_pose[0]-obst1_loc[0])/(d1**2),eta1*(bot_pose[1]-obst1_loc[1])/(d1**2)]
    else: 
        vec1 = [0,0]

    if (expr2 < 0):
        vec2 = [eta2*(bot_pose[0]-obst2_loc[0])/(d2**2),eta2*(bot_pose[1]-obst2_loc[1])/(d2**2)]
    else:
        vec2 = [0,0]

    if (expr3 < 0): 
        vec3 = [eta3*(bot_pose[0]-obst3_loc[0])/(d3**2),eta3*(bot_pose[1]-obst3_loc[1])/(d3**2)]
    else:
        vec3 = [0,0]

    if (expr4 < 0):
        vec4 = [eta4*(bot_pose[0]-obst4_loc[0])/(d4**2),eta4*(bot_pose[1]-obst4_loc[1])/(d4**2)]
    else:
        vec4 = [0,0]

    print(vec0,vec1)

    vec = [vec0[0]+vec1[0]+vec2[0]+vec3[0]+vec4[0], vec0[1]+vec1[1]+vec2[1]+vec3[1]+vec4[1]]
    mag = sqrt(vec[0]**2 + vec[1]**2)

    print(vec,mag)

    if (distance(bot_pose,goal_loc)<0.05):
        print("Reach goal location")
        print(bot_x)
        print(bot_y)
        bot_loc_plot(bot_x,bot_y)


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
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    

    #subscribe to odometry data to get the bot's pose
    subscribe_to_odom = rospy.Subscriber('/odom',Odometry, callback = odomdata_callback) 

    #create an object of Twist data

    move_the_bot = Twist()

    rospy.spin()


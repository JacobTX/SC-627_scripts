#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi, sqrt
import djikstra
import matplotlib.pyplot as plt


bot_pose = [] # Current pose of the bot
prev_pose = [100,100,0]

obst1_loc = [0,2]
obst2_loc = [1.5,0]
obst3_loc = [0,-2]
obst4_loc = [-1.5,0]

min_dist = 10 #minimum distance between bot and goal
min_loc = [] #location from which distance to goal is minimum
start_loc = [] #location from which circumnavigation starts
direc = [] #vector from bot to goal location


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

bot_x=[]
bot_y=[]

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

# Creating roadmap based on Voronoi diagram
O = (0,0) #start

entry_point=(0,0)
exit_point=(0,0)
entry_dist=100
exit_dist=100

# key points on the voronoi diagram

A = (-0.75,1)
B = (0.75,1)
C = (0.75,-1)
D = (-0.75,-1)
E = (0,0)
A1 = (-2.5,4)
B1 = (2.5,4)
C1 = (2.5,-4)
D1 = (-2.5,-4)
A2 = (0,4)
B2 = (2.5,0)
C2 = (0,-4)
D2 = (-2.5,0)

X = (-2.5,0) #goal


nodes = [O,A,B,C,D,E,A1,B1,C1,D1,A2,B2,C2,D2,X]	

init_graph = {}
for node in nodes:
    init_graph[node] = {}

for node in nodes[1:len(nodes)-1]:
    d_en = distance(list(O),list(node))
    d_ex = distance(list(X),list(node))
    if d_en < entry_dist:
        entry_dist = d_en
        entry_point = node
    if d_ex < exit_dist:
        exit_dist = d_ex
        exit_point = node

print("Entry", entry_point)
print("Exit", exit_point)
            

init_graph[O][entry_point] = distance(list(O),list(entry_point))

init_graph[A][E] = distance(list(A),list(E))
init_graph[B][E] = distance(list(B),list(E))
init_graph[C][E] = distance(list(C),list(E))
init_graph[D][E] = distance(list(D),list(E))
init_graph[A][A1] = distance(list(A),list(A1))
init_graph[B][B1] = distance(list(B),list(B1))
init_graph[C][C1] = distance(list(C),list(C1))
init_graph[D][D1] = distance(list(D),list(D1))
init_graph[A1][A2] = distance(list(A1),list(A2))
init_graph[A2][B1] = distance(list(A2),list(B1))
init_graph[B1][B2] = distance(list(B1),list(B2))
init_graph[B2][C1] = distance(list(B2),list(C1))
init_graph[C1][C2] = distance(list(C1),list(C2))
init_graph[C2][D1] = distance(list(C2),list(D1))
init_graph[D1][D2] = distance(list(D1),list(D2))
init_graph[D2][A1] = distance(list(D2),list(A1))

init_graph[X][exit_point] = distance(list(X),list(exit_point))


graph = djikstra.Graph(nodes, init_graph)
previous_nodes, shortest_path = djikstra.dijkstra_algorithm(graph=graph, start_node=O) 
path = djikstra.path(previous_nodes, start_node=O, target_node=X)
journey = path[::-1]

# END - Roadmap has been created

stage = 0

print(journey)
print("Moving from", journey[0],"to",journey[1])

def laserdata_callback(msg):

    global stage 
    a = journey[stage+1]

    vec = [a[0] - bot_pose[0], a[1] - bot_pose[1]]
    mag = sqrt(vec[0]**2 + vec[1]**2)

    if (distance(bot_pose,a) > 0.01):
        
        direc = [vec[0]/mag, vec[1]/mag]
        orient = [cos(bot_pose[2]),sin(bot_pose[2])]
        v = sgn(-direc[0]*orient[1] + direc[1]*orient[0])
        t = (direc[0]*orient[0]+direc[1]*orient[1])
        if v >= 0:
            p = acos(t)
        else:
            p = -acos(t)

        move_the_bot.linear.x = (0.05+0.05*sgn((0.01-p)*(0.01+p)))
        move_the_bot.angular.z = 0.1*v
        
    elif (distance(bot_pose,a) <= 0.01) and (stage < (len(journey)-2)):
        stage +=1 
        print("Moving from", journey[stage],"to",journey[stage+1])

    else:
        print("Reached")
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0

        bot_loc_plot(bot_x,bot_y)
        
       
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


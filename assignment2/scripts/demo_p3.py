#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi, sqrt, atan
import djikstra


bot_pose = [] # Current pose of the bot
goal_loc = [-0.5,-2.5] #Goal location
prev_pose = [100,100,0]

min_dist = 10 #minimum distance between bot and goal
min_loc = [] #location from which distance to goal is minimum
start_loc = [] #location from which circumnavigation starts
direc = [] #vector from bot to goal location


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

    
    # convert quaternion to yaw
    if (z >=0):
        theta = 2*acos(w)
    elif (z < 0):
        theta = 2*(pi - acos(w))

    global bot_pose
    bot_pose = [xloc,yloc,theta] 

# Obstacle class

class Obstacle:
    def __init__(self,O,l,b,yaw):
        self.O = O
        self.l = l
        self.b = b
        self.yaw = yaw
        self.d = sqrt(l**2 + b**2)/2
        self.alpha = 0.5*pi + yaw + atan(b/l)
        self.beta = 0.5*pi + yaw - atan(b/l)
        self.A = (O[0] +  self.d * cos(self.alpha), O[1] + self.d * sin(self.alpha))
        self.B = (O[0] +  self.d * cos(self.beta), O[1] + self.d * sin(self.beta))
        self.C = (O[0] -  self.d * cos(self.alpha), O[1] - self.d * sin(self.alpha))
        self.D = (O[0] -  self.d * cos(self.beta), O[1] - self.d * sin(self.beta)) 
        
        self.X = [self.A[0],self.B[0],self.C[0],self.D[0]]
        self.Y = [self.A[1],self.B[1],self.C[1],self.D[1]]
        

        i = self.X.index(min(self.X))
        j = self.X.index(max(self.X))
        k = self.Y.index(min(self.Y))
        l = self.Y.index(max(self.Y))


        

        self.extleft = (min(self.X),self.Y[i])
        self.extright = (max(self.X),self.Y[j])
        self.middown = (self.X[k],min(self.Y))
        self.midup = (self.X[l],max(self.Y))

        self.G1 = (self.extleft[0], 0.5*(self.extleft[1]+2.448))
        self.G2 = (self.extleft[0], 0.5*(self.extleft[1]-2.448))
        self.G3 = (self.midup[0],0.5*(self.midup[1]+2.448))
        self.G4 = (self.middown[0],0.5*(self.middown[1]-2.448))
        self.G5 = (self.extright[0], 0.5*(self.extright[1]+2.448))
        self.G6 = (self.extright[0], 0.5*(self.extright[1]-2.448))

        self.GX = [self.G1[0],self.G2[0],self.G3[0],self.G4[0],self.G5[0],self.G6[0]]
        self.GY = [self.G1[1],self.G2[1],self.G3[1],self.G4[1],self.G5[1],self.G6[1]]
        
        
O1 = (-2,-0.5)
l1 = 1
b1 = 0.5339
yaw1 = -0.5
Ob1 = Obstacle(O1,l1,b1,yaw1)

O2 = (0.5,1)
l2 = 1
b2 = 0.592
yaw2 = -2.615
Ob2 = Obstacle(O2,l2,b2,yaw2)

O3 = (2.5,0)
l3 = 1
b3 = 1
yaw3 = -0.3075
Ob3 = Obstacle(O3,l3,b3,yaw3)

# Creating roadmap based on trapezoidal decomposition

O = (-3.5,-0.5) #start

entry_point = (0,0)
exit_point = (0,0)
entry_dist = 100
exit_dist = 100


P0 = (-4.36, 0.7906)

G11 = Ob1.G1
G12 = Ob1.G2
G13 = Ob1.G3
G14 = Ob1.G4
G15 = Ob1.G5
G16 = Ob1.G6

G21 = Ob2.G1
G22 = Ob2.G2
G23 = Ob2.G3
G24 = Ob2.G4
G25 = Ob2.G5
G26 = Ob2.G6

G31 = Ob3.G1
G32 = Ob3.G2
G33 = Ob3.G3
G34 = Ob3.G4
G35 = Ob3.G5
G36 = Ob3.G6

P01 = (2,-1)
P12 = (0.25*(G15[0]+G16[0]+G21[0]+G22[0]),0.25*(G15[1]+G16[1]+G21[1]+G22[1]))
P23 = (0.25*(G25[0]+G26[0]+G31[0]+G32[0]),0.25*(G25[1]+G26[1]+G31[1]+G32[1]))
P34 = (3.873, -1)

P4 = (5,-1)

X = (0,0) #goal

nodes = [O,P0,G11,G12,G13,G14,G15,G16,G21,G22,G23,G24,G25,G26,G31,G32,G33,G34,G35,G36,P01,P12,P23,P34,P4,X]	

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

init_graph[P0][P01] = distance(list(P0),list(P01))
init_graph[P01][G11] = distance(list(P01),list(G11))
init_graph[P01][G12] = distance(list(P01),list(G12))
init_graph[G11][G13] = distance(list(G11),list(G13))
init_graph[G12][G14] = distance(list(G12),list(G14))
init_graph[G13][G15] = distance(list(G13),list(G15))
init_graph[G14][G16] = distance(list(G14),list(G16))
init_graph[G15][P12] = distance(list(G15),list(P12))
init_graph[G16][P12] = distance(list(G16),list(P12))
init_graph[P12][G21] = distance(list(P12),list(G21))
init_graph[P12][G22] = distance(list(P12),list(G22))
init_graph[G21][G23] = distance(list(G21),list(G23))
init_graph[G22][G24] = distance(list(G22),list(G24))
init_graph[G23][G25] = distance(list(G23),list(G25))
init_graph[G24][G26] = distance(list(G24),list(G26))
init_graph[G25][P23] = distance(list(G25),list(P23))
init_graph[G26][P23] = distance(list(G26),list(P23))
init_graph[P23][G31] = distance(list(P23),list(G31))
init_graph[P23][G32] = distance(list(P23),list(G32))
init_graph[G31][G33] = distance(list(G31),list(G33))
init_graph[G32][G34] = distance(list(G32),list(G34))
init_graph[G33][G35] = distance(list(G33),list(G35))
init_graph[G34][G36] = distance(list(G34),list(G36))
init_graph[G35][P34] = distance(list(G35),list(P34))
init_graph[G36][P34] = distance(list(G36),list(P34))
init_graph[P34][P4] = distance(list(P34),list(P4))

init_graph[X][exit_point] = distance(list(X),list(exit_point))


graph = djikstra.Graph(nodes, init_graph)
previous_nodes, shortest_path = djikstra.dijkstra_algorithm(graph=graph, start_node=O) 
path = djikstra.path(previous_nodes, start_node=O, target_node=X)
journey = path[::-1]

# END - Roadmap has been created and short path has been determined

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
        
    elif (distance(bot_pose,a) <= 0.01) and (stage < len(journey)-2):
        stage +=1 
        print("Moving from", journey[stage],"to",journey[stage+1])

    else:
        print("Reached")
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0
        
       
    publish_to_cmd_vel.publish(move_the_bot) 
    

if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/tb3_1/scan', LaserScan, callback = laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size = 10)
    

    #subscribe to odometry data to get the bot's pose
    subscribe_to_odom = rospy.Subscriber('/tb3_1/odom',Odometry, callback = odomdata_callback) 

    #create an object of Twist data

    move_the_bot = Twist()

    rospy.spin()


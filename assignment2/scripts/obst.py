#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, acos, pi, sqrt, atan
import matplotlib.pyplot as plt


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

x = Ob1.X + Ob2.X + Ob3.X 
y = Ob1.Y + Ob2.Y + Ob3.Y 
gx = Ob1.GX + Ob2.GX + Ob3.GX 
gy = Ob1.GY + Ob2.GY + Ob3.GY 

plt.scatter(x+gx,y+gy)
plt.xlim([-4,4])
plt.ylim([-4,4])
plt.show()
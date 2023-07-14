import math
import time

from joint_pub import legjoints
from joint_pub import point_finder

# function for finding roots
def QuadSolver( a:float, b:float, c:float):
 
    # calculating discriminant using formula
    dis = b * b - 4 * a * c
    sqrt_val = math.sqrt(abs(dis))
     
    # checking condition for discriminant
    if dis > 0:
        print("real and different roots")
        # print((-b + sqrt_val)/(2 * a))
        # print((-b - sqrt_val)/(2 * a))
        return(((-b + sqrt_val)/(2 * a),(-b - sqrt_val)/(2 * a)))
     
    elif dis == 0:
        print("real and same roots")
        # print(-b / (2 * a))
        return((-b / (2 * a),-b / (2 * a)))
     
    # when discriminant is less than 0
    else:
        print("Complex Roots")
        # print(- b / (2 * a), + i, sqrt_val)
        # print(- b / (2 * a), - i, sqrt_val)

# t2max ,t2 min in degrees

class optimizer:
    def __init__(self,height:float,leg1:legjoints,theta2_max:int=180,theta2_min:int=0) -> None:
        self.leg=leg1
        self.t2max=math.radians(theta2_max)
        self.t2min=math.radians(theta2_min)
        self.h=height
        self.t1max=None
        self.t1min=None
        self.r_major=None
        self.r_minor=None

    def max_length_finder(self):

        print('Finding t1max')
        l1=self.leg.thigh_len
        l2=self.leg.ankle_len
        a=(l1+(l2*math.cos(self.t2max)))
        b=l2*math.sin(self.t2max)
        c1sqr=(b**2)/((a**2)+(b**2))
        s1sqr=1-c1sqr
        self.t1max=math.atan2(math.sqrt(s1sqr),math.sqrt(c1sqr))
        print(self.t1max*180/3.14)           #t1 coresponding to max value of t2 not max value of t1

        print('Finding t1min')
        a=l1+(l2*math.cos(self.t2min))
        b=l2*math.sin(self.t2min)
        solns=QuadSolver((a**2)+(b**2),-2*self.h*a,(self.h**2)-(b**2))
        if(solns[0]>=0):
            s1=solns[0]         #theta1 in 1st or 2nd Quadrant so sin positive
        elif(solns[1]>=0):
            s1=solns[1]
        else:
            print('Error s1sqr not found')
            return
        s1sqr=s1**2
        c1sqr=1-s1sqr
        self.t1min=math.atan2(math.sqrt(s1sqr),math.sqrt(c1sqr))
        self.t1min=math.radians(180)-self.t1min  #converting to clockwise angle that we need
        print(self.t1min*180/3.14)       #t1 for min value of t2

        print('Finding r values')
        print(l1*math.cos(self.t1min))
        print(l2*math.cos(self.t1min+self.t2min))
        self.r_major=(l1*math.cos(self.t1min))+(l2*math.cos(self.t1min+self.t2min))
        self.r_minor=self.h-((l1*math.sin(self.t1max))+(l2*math.sin(self.t1max+self.t2max)))
        print('Max path length: ',self.r_major)
        print('Max height:   ',self.r_minor)
        return self.r_major,self.r_minor
    

my_leg=legjoints()
opt=optimizer(9,my_leg,130,45)
print(opt.max_length_finder())
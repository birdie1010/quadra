import math
import time
import numpy as np
from pprint import pprint
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
    
#1 is hip and 2 is knee
#first letter is super script and second one is sub
class calculator:
    def __init__(self,leg1:legjoints) ->None:
        self.l1=leg1.thigh_len
        self.l2=leg1.ankle_len
    
    def vel_calc_mat(self,t1:float,t2:float,w1:float,w2:float):
        s2=math.sin(t2)
        c2=math.cos(t2)
        self.r21=np.matrix(f'{c2} {-s2} 0 {self.l1} ; {s2} {c2} 0 0 ; 0 0 1 0 ; 0 0 0 1',float)  #rotation matrix 1 to 2
        self.w11=np.matrix(f'0;0;{w1} ;1',float)
        self.w11c=self.w11
        self.v11=np.matrix('0 ; 0 ; 0 ;1',float)
        w21=np.dot(self.r21,self.w11)
        self.w22=np.add(w21,np.matrix(f'0;0;{w2};0',float))  #w2c would be equal to this one. In the equation w2 is theta2 dot
        w11_vect=np.matrix(f'0 0 {w1}',float)   #w11 vector to be used in cross products.4 dimesional cross products D.N.E. So cannot be done in homogenous
        self.v11c=np.cross(w11_vect,np.matrix(f'{self.l1/2} 0 0'))
        v12=np.cross(w11_vect,np.matrix(f'{self.l1} 0 0'))
        v12_homo=np.append(v12,[1])
        self.v22=np.dot(self.r21,v12_homo)
        w22_vect=np.matrix(f'{self.w22[0]} {self.w22[1]} {self.w22[2]}')
        v22c_p1=np.cross(w22_vect,np.matrix(f'{self.l1} 0 0',float))
        v22_vect=np.matrix(f'{self.v22.item(0,0)} {self.v22.item(0,1)} {self.v22.item(0,2)}')
        self.v22c=np.add(v22c_p1,v22_vect)          #in vector form
        # pprint(vars(self))  

    def acce_calc(self,a1:float,a2:float,w1:float,w2:float):
        self.alpha11=np.matrix(f'0 ;0 ;{a1}; 1')
        alpha22_p1=np.dot(self.r21,self.alpha11)
        alpha22_p1_vect=np.matrix(f'{alpha22_p1.item(0,0)} {alpha22_p1.item(1,0)} {alpha22_p1.item(2,0)}')
        w11_vect=np.matrix(f'{self.w11.item(0,0)} {self.w11.item(1,0)} {self.w11.item(2,0)}')
        self.a1=np.matrix(f'0 0 0 0')
        w21=np.dot(self.r21,self.w11)
        w21_vect=np.matrix(f'{w21.item(0,0)} {w21.item(1,0)} {w21.item(2,0)}')
        alpha22_p2=np.cross(np.matrix(f'0 0 {w2}'),w21_vect)
        alpha22_p3=np.matrix(f'0 0 {a2}')
        self.alpha22=np.add(alpha22_p1_vect,np.add(alpha22_p2,alpha22_p3))
        self.a11c=np.cross(w11_vect,np.matrix(f'{self.l1/2} 0 0'))
        temp=np.cross(w11_vect,np.matrix(f'{self.l1} 0 0'))
        temp_homo=np.append(temp,[1])
        a22_p1=np.dot(self.r21,temp_homo)
        # theta2_dot=np.matrix(f'0 0 {w2}')
        a22_p2_a=np.cross(w11_vect,temp)
        a22_p2=np.dot(self.r21,np.append(a22_p2_a,[1]))
        a22_p3=np.squeeze(np.dot(self.r21,self.a1.T))
        a22=np.add(a22_p1,np.add(a22_p2,a22_p3))
        self.a22_vect=np.matrix(f'{a22.item(0)} {a22.item(1)} {a22.item(2)}')
        temp=np.cross(np.matrix(f'{self.w22.item(0)} {self.w22.item(1)} {self.w22.item(2)}'),np.matrix(f'{self.l2/2} 0 0'))
        a22c_p2=np.cross(np.matrix(f'{self.w22.item(0)} {self.w22.item(1)} {self.w22.item(2)}'),temp)
        self.a22c=np.add(temp,np.add(a22c_p2,self.a22_vect))
        # a22_p2_b
        pprint(vars(self))


my_leg=legjoints()
# opt=optimizer(9,my_leg,130,45)
# print(opt.max_length_finder())
number=3
my_leg.points=point_finder('circle',(0,9),1,number)
my_leg.inv_kin_list()
calc=calculator(my_leg)
vel1_list=np.diff([i[0] for i in my_leg.angles])
vel2_list=np.diff([i[1] for i in my_leg.angles])
acel1_list=np.diff([i[0] for i in my_leg.angles],n=2)
acel2_list=np.diff([i[1] for i in my_leg.angles],n=2)
# print('leg angles',my_leg.angles)
# print('vel1_list',vel1_list)
# print('vel2_list',vel2_list)
# print('acel1_list',acel1_list)
# print('acel2_list',acel2_list)
for i in range(number-2):    #2 less since accel,vel needed
    calc.vel_calc_mat(my_leg.angles[i][0],my_leg.angles[i][1],vel1_list[i],vel2_list[i])
    # print('Velocity done')
    calc.acce_calc(acel1_list[i],acel2_list[i],vel1_list[i],vel2_list[i])
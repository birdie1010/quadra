import math
import time

def fd_kin_single(angles,len1,len2):
        l1=len1
        l2=len2
        x=(l1*math.cos(-angles[0]))+(l2*math.cos(-angles[0]+angles[1]))
        y=(l1*math.sin(-angles[0]))+(l2*math.sin(-angles[0]+angles[1]))
        return (round(x,4),round(y,4))


def inv_kin_single(point,len1,len2):
        x=point[0]
        y=point[1]
        c2=((x**2)+(y**2)-(len1**2)-(len2**2))/(2*len1*len2)
        # print('c2 ',c2)
        if(c2<=1 and c2>=-1):
            theta2=math.acos(c2)
            # s2=math.sqrt(1-(c2**2))
            # theta2=math.atan2(s2,c2)
        else:
            print(f'Cos value error for c2 on pt ({x},{y})')
            return None
        A=len1+(len2*c2)
        B=(len2*math.sin(theta2))
        c1=((A*x)+(B*y))/((A**2)+(B**2))
        # print('c1 ',c1)
        if(c1<=1 and c1>=-1):
            theta1=-math.acos(c1)   #require value in third or fourth quad. So -ve of ans of arccos. Its ans in 1st or 2nd quad
            # print('t1 ',theta1*180/3.14)
        else:
            print('Cos value error for hip')
            return None
        if(theta1 != None and theta2 != None):
            # (self.hip_state_pos,self.knee_state_pos)=(theta1,theta2)
            return (theta1,theta2)
        
print('inv ans',inv_kin_single((1.5,11.5),10,2))
print('fwd ans',fd_kin_single((-1.329,0.703),10,2))
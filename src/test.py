import math
import time

def inv_kin_single(point,len1,len2):
        x=point[0]
        y=point[1]
        c2=((x**2)+(y**2)-(len1**2)-(len2**2))/(2*len1*len2)
        # print(c2)
        if(c2<=1 and c2>=-1):
            s2=math.sqrt(1-(c2**2))
            theta2=math.atan2(s2,c2)
        else:
            print('Cos value error for knee')
            return None
        A=len1+(len2*c2)
        B=(len2*s2)
        c1=((A*x)+(B*y))/((A**2)+(B**2))
        # print(c1)
        if(c1<=1 and c1>=-1):
            s1=math.sqrt(1-(c1**2))        
            # print(s1)
            if(x>=0):  #quadrant 4 tan negative
                theta1=math.atan2(s1,-c1)
            else:
                theta1=math.atan2(s1,c1)
        else:
            print('Cos value error for hip')
            return None
        if(theta1 != None and theta2 != None):
            # (self.hip_state_pos,self.knee_state_pos)=(theta1,theta2)
            return (theta1,theta2)
        
print(inv_kin_single((1.5,11.5),10,2))
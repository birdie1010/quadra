import math
PI=3.14
class legjoints:
    thigh_len=0.5
    ankle_len=0.5
    points=[]
    angles=[]
    def __init__(self):
        # self.name=leg_number
        self.hip_state_pos=PI/6
        self.knee_state_pos=-PI/6
        self.hip_state_vel=0
        self.knee_state_vel=0
        self.step_index=0

    def inv_kin_single(self,point):
        x=point[0]
        y=point[1]
        l1=self.thigh_len
        l2=self.ankle_len
        c2=((x**2)+(y**2)-(l1**2)-(l2**2))/(2*l1*l2)
        if(c2<=1 and c2>=-1):
            s2=math.sqrt(1-(c2**2))
            theta2=math.atan2(s2,c2)
        else:
            print('Cos value error for knee')
            return None
        A=l1+(l2*c2)
        B=(l2*s2)
        c1=((A*x)+(B*y))/((A**2)+(B**2))
        if(c1<=1 and c1>=-1):
            s1=math.sqrt(1-(c1**2))
            theta1=math.atan2(s1,c1)
        else:
            print('Cos value error for hip')
            return None
        if(theta1 != None and theta2 != None):
            # (self.hip_state_pos,self.knee_state_pos)=(theta1,theta2)
            return (theta1,theta2)
# print(point_finder('circle',(1,1),2,5))
    def inv_kin_list(self):
        angles=[]
        for i in range(len(points)-1):
            angles.append(self.inv_kin_single(self.points[i]))
        self.angles=angles
joint=legjoints()
print(joint.inv_kin_single((math.sin(math.radians(45)),0)))
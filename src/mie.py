#!/usr/bin/env python
# license removed for brevity
PI=3.14
import rospy
from sensor_msgs.msg import JointState
import time
import math

# a function to give the required number of pts as a list
def point_finder(path_type,foci,radius,number_of_pts):
    division=math.radians(180)/(number_of_pts-1)
    point_list=[]
    if(number_of_pts<2):
        print('Error Min no of pts is 2')
    if(path_type.lower()=='circle'): 
        theta=0
        point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))       
        point_list.append((round(point[0],4),round(point[1],4)))
        while(theta != math.radians(180)):
            theta+=division
            point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))       
            point_list.append((round(point[0],4),round(point[1],4)))
    else:
        print('Error give proper path type')
    return point_list
            

class legjoints:
    thigh_len=5
    ankle_len=5
    points=[]
    angles=[]
    def __init__(self):
        # self.name=leg_number
        self.hip_state_pos=PI/6
        self.knee_state_pos=-PI/6
        self.hip_state_vel=0
        self.knee_state_vel=0
        self.step_index=0
    def fd_mv_up(self):
        if(self.step_index==0):
            if(self.hip_state_pos>=-PI/3):
                self.hip_state_pos-=0.05
            if(self.knee_state_pos<=PI/2):
                self.knee_state_pos+=0.05
            elif(self.hip_state_pos<=-PI/3):
                self.step_index=1
        elif(self.step_index==1):
            if(self.hip_state_pos<=PI/6):
                self.hip_state_pos+=0.05
            else:
                self.step_index=2
        elif(self.step_index==2):
            if(self.knee_state_pos>=-PI/6):
                self.knee_state_pos-=0.05
    
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

    def inv_kin_list(self):
        angles=[]
        for i in range(len(self.points)-1):
            angles.append(self.inv_kin_single(self.points[i]))
        self.angles=angles
        
        
    def fd_mv_dwn(self):
        pass



legs=legjoints()
def talker():
    global legs,stepindex
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(2) # 10hz
    #finding angles
    legs.points=point_finder('circle',(7.5,0),2.0,15)
    legs.inv_kin_list()
    print('Points : ',legs.points)
    print('angles : ',legs.angles)    
    index=0

    while not rospy.is_shutdown():
        # joints.fd_mv_up()

        joint_states=JointState()
        joint_states.header.stamp=rospy.get_rostime()

        #pulishing below this
        # joint_states.name.append('rota')
        # joint_states.position.append(legs.hip_state_pos)
        # joint_states.name.append('knee')
        # joint_states.position.append(legs.knee_state_pos)

        if(index<len(legs.points)-1):
            # print(legs.angles[index][0])
            joint_states.name.append('rota')
            joint_states.position.append(legs.angles[index][0])
            joint_states.name.append('knee')
            joint_states.position.append(legs.angles[index][1])
            index+=1
            pub.publish(joint_states)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

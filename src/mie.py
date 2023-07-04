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
        
        
    def fd_mv_dwn(self):
        pass



joints=legjoints()
def talker():
    global joints,stepindex
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        joint_states=JointState()
        joints.fd_mv_up()
        # rospy.loginfo(hello_str)
        joint_states.header.stamp=rospy.get_rostime()
        # joint_states.name.resize(1)
        

        # joint_states.position.resize(1)
        joint_states.name.append('rota')
        joint_states.position.append(joints.hip_state_pos)
        joint_states.name.append('knee')
        joint_states.position.append(joints.knee_state_pos)

        pub.publish(joint_states)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

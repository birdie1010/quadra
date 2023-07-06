#!/usr/bin/env python
# license removed for brevity
PI=3.14
import rospy
from sensor_msgs.msg import JointState
import time
import math

num_of_pt=15
leg_travel_dist=4
# a function to give the required number of pts as a list
#give theta in degrees for lines. Otherwise not required.give it in angle from -180 to 180.
def point_finder(path_type,foci,radius,number_of_pts,theta=None):
    point_list=[]
    if(number_of_pts<2):
        print('Error Min no of pts is 2')
        return None
    if(path_type.lower()=='circle'): 
        division=math.radians(180)/(number_of_pts-1)
        theta=math.radians(180)
        point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))       
        point_list.append((round(point[0],4),round(point[1],4)))
        while(theta != math.radians(0)):
            theta-=division
            point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))       
            point_list.append((round(point[0],4),round(point[1],4)))
    elif(path_type.lower()=='linear'):
        ini_pt=foci
        length=0
        if(math.isnan(theta)):
            print('Error give direction of line')
            return None
        theta=math.radians(theta)
        division=radius/number_of_pts  #radius used as total length of line
        print(division)
        point_list.append(ini_pt)
        for i in range(number_of_pts):
            length+=division
            length=round(length,4)  #binary calculation error is coming if round not included
            # print(length)
            # time.sleep(0.5)
            point=(ini_pt[0]+(length*math.cos(theta)),ini_pt[1]+(length*math.sin(theta)))
            point_list.append((round(point[0],4),round(point[1],4)))
    else:
        print('Error give proper path type')
        return None
    return point_list
            

class legjoints:
    global num_of_pt,leg_travel_dist
    thigh_len=5
    ankle_len=5
    points=[]
    angles=[]
    def __init__(self,leg_no=1):
        self.leg_no=leg_no
        self.hip_state_pos=PI/6
        self.knee_state_pos=-PI/6
        self.hip_state_vel=0
        self.knee_state_vel=0
        self.step_index=0
        self.position_no=0
    def fd_mv_up(self):
        #each call moves to next position
        global joint_states
        if(self.position_no==0):
            self.points=point_finder('circle',(0,7.5),leg_travel_dist/2,num_of_pt)
            self.inv_kin_list()
            print('Points : ',self.points)
            print('angles : ',self.angles)    
        if(self.position_no<len(self.angles)):
            joint_states.name.append(f'rota_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][0])
            joint_states.name.append(f'knee_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][1])
            print(joint_states)
            if(self.position_no<len(self.angles)-1):
                self.position_no+=1
        else:
            return        

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
            return (theta1,-theta2)

    def inv_kin_list(self):
        angles=[]
        for i in range(len(self.points)):
            angles.append(self.inv_kin_single(self.points[i]))
        self.angles=angles

    def fd_kin_single(self,angles):
        l1=self.thigh_len
        l2=self.ankle_len
        x=(l1*math.cos(angles[0]))+(l2*math.cos(angles[0]+angles[1]))
        y=(l1*math.sin(angles[0]))+(l2*math.sin(angles[0]+angles[1]))
        return ((round(x,4),round(y,4)))

    def fd_mv_dwn(self):
        global joint_states
        if(self.position_no==0):
            self.points=point_finder('linear',(0,7.5),leg_travel_dist/2,num_of_pt,180)
            self.inv_kin_list()
            print('Points : ',self.points)
            print('angles : ',self.angles)    
        if(self.position_no<len(self.angles)):
            joint_states.name.append(f'rota_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][0])
            joint_states.name.append(f'knee_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][1])
            print(joint_states)
            if(self.position_no<len(self.angles)-1):
                self.position_no+=1
        else:
            return        



leg1=legjoints(1)
leg2=legjoints(2)
leg3=legjoints(3)
leg4=legjoints(4)
joint_states=JointState()

def talker():
    global leg1,leg2,leg3,leg4,joint_states,num_of_pt
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(2) # 10hz
    
    while not rospy.is_shutdown():     
        joint_states.name.clear()
        joint_states.position.clear()
        joint_states.header.stamp=rospy.get_rostime()

        # pulishing below this axis trail
        # for i in range(1,5):            
        #     joint_states.name.append(f'rota_{i}')
        #     joint_states.position.append(math.radians(0))
        #     joint_states.name.append(f'knee_{i}')
        #     joint_states.position.append(math.radians(0))
        # pub.publish(joint_states)
        # rate.sleep()

        i=0
        if(i<num_of_pt):
            i+=1
            leg1.fd_mv_up()
            leg2.fd_mv_dwn()
            leg3.fd_mv_up()
            leg4.fd_mv_dwn()
            pub.publish(joint_states)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

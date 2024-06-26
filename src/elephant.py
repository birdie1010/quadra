#!/usr/bin/env python
# license removed for brevity
PI=3.14
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
# from std_msgs.msg import Empty
import time
import math

num_of_pt=100        #should be even
leg_travel_dist=1.561    #cannot be independent should be dep on other roboheight. So chnaged later
robo_height=9
# a function to give the required number of pts as a list
#give theta in degrees for lines. Otherwise not required.give it in angle from -180 to 180.
def point_finder(path_type,foci,radius,number_of_pts,theta=None):
    point_list=[]
    if(number_of_pts<2):
        print('Error Min no of pts is 2')
        return None
    if(path_type.lower()=='circle'): 
        division=math.radians(180)/(number_of_pts-1)
        theta=-math.radians(180)
        point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))  
        # print(point)     
        point_list.append((round(point[0],4),round(point[1],4)))
        for i in range(number_of_pts-1):
            theta+=division
            point=(foci[0]+(radius*math.cos(theta)),foci[1]+(radius*math.sin(theta)))    
            # print(point)   
            point_list.append((round(point[0],4),round(point[1],4)))
    elif(path_type.lower()=='linear'):
        ini_pt=foci
        length=0
        if(math.isnan(theta)):
            print('Error give direction of line')
            return None
        theta=math.radians(theta)
        division=radius/number_of_pts  #radius used as total length of line
        # print(division)
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

def ori_finder(ini,mid,fin):
    global num_of_pt
    division=(mid-ini)*2/(num_of_pt-1)
    orient=[]

    for i in range(int(num_of_pt/2)):
        orient.append(ini+(division*i))

    division=(fin-mid)*2/(num_of_pt-1)
    for i in range(int(num_of_pt/2)-1):
        orient.append(mid+(division*i))

    orient.append(fin)

    return orient


class legjoints:
    global num_of_pt,leg_travel_dist,robo_height
    l1=6
    l2=4
    l3=2
    
    def __init__(self,leg_pos,leg_no=1):
        global leg_travel_dist
        self.points=[]
        self.angles=[]
        self.leg_pos=leg_pos
        self.leg_no=leg_no
        self.hip_state_pos=PI/6
        self.knee_state_pos=-PI/6
        self.hip_state_vel=0
        self.knee_state_vel=0
        self.step_index=0
        self.position_no=0
        self.posi=None
        leg_travel_dist=self.ltd_finder(robo_height)

    def fd_mv_up(self,circ_radius=None,center=None):
        #each call moves to next position
        global joint_states
        leg_pos=self.leg_pos
        if(self.position_no==0):
            self.points.clear()
            self.angles.clear()
            if(circ_radius==None):
                self.points=point_finder('circle',(0,robo_height-self.l3),leg_travel_dist,num_of_pt)      #circle of diameter 4ltd required. Because duty factor is only 0.25(approx)
            elif(center==None):
                self.points=point_finder('circle',(circ_radius,robo_height-self.l3),circ_radius,num_of_pt)  #for initializing gate a smaller step required
            else:
                self.points=point_finder('circle',center,circ_radius,num_of_pt)  #for turns
            # print(f'Points found Fd mv up for leg{self.leg_no}')
            # print('Points : ',self.points)
            self.inv_kin_list_3link(self.l1,self.l2)
            # print('Angles found Fd mv up')
            # print('angles : ',self.angles)    
        if(self.position_no<len(self.angles)):
            # print(f'insisde fd mv up {self.leg_no}')
            joint_states.name.append(f'hip_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][0])
            joint_states.name.append(f'knee_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][1])
            joint_states.name.append(f'ankle_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][2])
            # print(joint_states)
            # if(self.position_no<len(self.angles)-1):
            # print(self.position_no)
            self.position_no+=1
            self.position_no%=num_of_pt
        else:
            return       

    def inv_kin_single(self,point,len1,len2):
        x=point[0]
        y=point[1]
        c2=((x**2)+(y**2)-(len1**2)-(len2**2))/(2*len1*len2)
        if(c2<=1 and c2>=-1):
            theta2=math.acos(c2)
            # s2=math.sqrt(1-(c2**2))
            # theta2=math.atan2(s2,c2)
        else:
            print(f'Cos value error for c2 on pt ({x},{y}) for {self.leg_no}')
            return None
        A=len1+(len2*c2)
        B=(len2*math.sin(theta2))
        c1=((A*x)+(B*y))/((A**2)+(B**2))
        # print(math.acos(c1)*180/3.14)
        if(c1<=1 and c1>=-1):
            theta1=-math.acos(c1)   #require value in third or fourth quad. So -ve of ans of arccos. Its ans in 1st or 2nd quad
            # print(theta1*180/3.14)
        else:
            print('Cos value error for hip')
            return None
        if(theta1 != None and theta2 != None):
            # (self.hip_state_pos,self.knee_state_pos)=(theta1,theta2)
            return (theta1,theta2)
        
    def inv_kin_list_3link(self,len1:float,len2:float):
        leg_pos=self.leg_pos
        inih,inik=self.inv_kin_single(self.points[0],len1,len2)
        finh,fink=self.inv_kin_single(self.points[-1],len1,len2)
        if(leg_pos=='hind'):
            ori=ori_finder(-(inih-inik)-1.57,1,-(finh-fink)-1.57)
            # print('value',inih,inik,finh,fink)
            # print('ori',ori)
            # print(len(ori),len(self.points))
            for i in range(len(self.points)):
                hip,knee=self.inv_kin_single(self.points[i],len1,len2)
                self.angles.append((hip,-knee,ori[i]))
            self.posi=(self.points[-1][0],self.points[-1][1]+self.l3)

        elif(leg_pos=='front'):
            ori=ori_finder(1.57+inih-inik,-1.57,1.57+finh-fink)
            # print('value',inih,inik,finh,fink)
            # print('ori',ori)
            # print(len(ori),len(self.points))
            self.points.reverse()
            for i in range(len(self.points)):
                hip,knee=self.inv_kin_single(self.points[i],len1,len2)
                self.angles.append((3.14-hip,knee,ori[i]))
            # self.angles.reverse()
            self.posi=(self.points[-1][0],self.points[-1][1]+self.l3)
        # print(f'posi for {self.leg_no}',self.posi)

    def inv_kin_list(self,len1:float,len2:float):
        leg_pos=self.leg_pos
        if(leg_pos=='hind'):
            # print(f'back pos {self.leg_no} ',self.points)
            self.posi=(self.points[-1][0],self.points[-1][1])
            for i in range(len(self.points)):
                # print('hind called')
                hip,ankle=self.inv_kin_single(self.points[i],len1,len2)
                self.angles.append((hip,0,ankle))
        elif(leg_pos=='front'):
            # print(f'front pos {self.leg_no}',self.points)
            self.posi=(self.points[-1][0],self.points[-1][1])
            # self.points.reverse()
            for i in range(len(self.points)):
                hip,ankle=self.inv_kin_single(self.points[i],len1,len2)
                self.angles.append((3.14-hip,0,-ankle))
        # print(self.posi)
            # self.angles.reverse()

    def fd_kin_single(self,angles):
        # print(angles)
        x=(self.l1*math.cos(angles[0]))+(self.l2*math.cos(angles[0]+angles[1]))+(self.l3*math.cos(angles[0]+angles[1]+angles[2]))
        y=(self.l1*math.sin(angles[0]))+(self.l2*math.sin(angles[0]+angles[1]))+(self.l3*math.cos(angles[0]+angles[1]+angles[2]))
        print(x,y)
        return (round(x,4),round(y,4))

    def fd_mv_dwn(self,circ_radius=None):
        leg_pos=self.leg_pos
        position_h=(0,-robo_height)
        position_v=(0,-robo_height)
        global joint_states
        if self.posi:
            position_v=self.posi
            position_h=self.posi
            # print(f'Position set for {self.leg_no}')
        if(self.position_no==0):
            self.points.clear()
            self.angles.clear()
            if(circ_radius==None):
                if(self.leg_pos=='hind'):
                    self.points=point_finder('linear',position_h,leg_travel_dist*2/3,num_of_pt,180)
                elif self.leg_pos=='front':
                    self.points=point_finder('linear',position_v,leg_travel_dist*2/3,num_of_pt,0)
                    # self.points.reverse()
                # print(f'Points by mv down of {self.leg_no}',self.points)
            else:
                if(self.leg_pos=='hind'):
                    self.points=point_finder('linear',position_h,circ_radius,num_of_pt,180)
                elif self.leg_pos=='front':
                    self.points=point_finder('linear',position_v,circ_radius,num_of_pt,0)
            # print(f'Points for {self.leg_no} : ',self.points)
            self.inv_kin_list(self.l1+self.l2,self.l3)
            # print(self.angles)
            # self.inv_kin_list(2,10,2)
            # print('angles : ',self.angles)    
        if(self.position_no<len(self.angles)):
            # print(f'in fd mv dwn {self.leg_no}')
            joint_states.name.append(f'hip_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][0])
            joint_states.name.append(f'knee_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][1])
            joint_states.name.append(f'ankle_{self.leg_no}')
            joint_states.position.append(self.angles[self.position_no][2])
            # print(joint_states.name)
            # if(self.position_no<len(self.angles)-1):
            self.position_no+=1
            self.position_no%=num_of_pt
        else:
            return        

    def ltd_finder(self,height:float) ->float:
        a=(self.l1+self.l2)**2
        b=(height-self.l3)**2
        l=math.sqrt((a-b))-0.001       #-0.001 for safety otherwise cos may show error.  Arccos(1.00000001) may occur
        # print(a,b)
        # print('leg travel dist',l)
        return l

class move_fns:
    global num_of_pt,leg_travel_dist,pub,rate,robo_height,joint_states
    def __init__(self,l1:legjoints,l2:legjoints,l3:legjoints,l4:legjoints) -> None:
        self.legs=[l1,l2,l3,l4]
        self.i=0
        self.cycle=0
        self.prev_cycle=None
        self.stop=False     #true only if robot at rest

    def gait(self,gait_type):
        self.contin=rospy.get_param('/contin_walk',True)
        if(gait_type.lower()=='amble'):
            if(self.contin or self.prev_cycle==self.cycle):
                self.stop=False  #if comming in after stopping this is required
                gait_pos=(self.cycle%4)
                if(self.cycle<3):      #first step is a smaller step to initialize gait
                    self.gait_init()
                    # pass
                else:
                    for j in range(4):
                        # print(self.i,gait_pos,j)
                        if(gait_pos==j):
                            # print(f'leg{self.legs[j].leg_no} moving up')
                            self.legs[j].fd_mv_up()
                        else:
                            # pass
                            # print(f'leg {j} moving down')
                            self.legs[j].fd_mv_dwn()

                self.prev_cycle=self.cycle
                self.i+=1
                self.cycle=int(self.i/num_of_pt)        #this is cycle number not point number
                # print(self.i)
            # elif(self.stop==False):
            #     #to stop trot gait and come to rest
            #     # for i in range(num_of_pt):
            #     if(self.cycle%2==0):                
            #         self.leg1.fd_mv_up(leg_travel_dist/4)
            #         self.leg3.fd_mv_up(leg_travel_dist/4)
            #     else:
            #         self.leg2.fd_mv_up(leg_travel_dist/4)
            #         self.leg4.fd_mv_up(leg_travel_dist/4)
            #     self.prev_cycle=None
            #     self.i+=1
            #     # self.cycle=int(self.i/num_of_pt)        #this is cycle number not point number
            #     if(self.cycle!=int(self.i/num_of_pt)):
            #         self.cycle=0        #this is cycle number not point number
            #         self.stop=True
            #         self.i=0
            #     print('stopping')
    def gait_init(self):
        if(self.cycle==0):
            self.legs[0].fd_mv_up(3*leg_travel_dist/6)
            self.legs[1].fd_mv_dwn(2*leg_travel_dist/3)
            self.legs[2].fd_mv_dwn(leg_travel_dist/3)
            self.legs[3].fd_mv_dwn(leg_travel_dist/3)
        elif(self.cycle==1):
            self.legs[0].fd_mv_dwn()
            self.legs[1].fd_mv_up(4*leg_travel_dist/6,(-leg_travel_dist/6,robo_height-self.legs[1].l3))
            self.legs[2].fd_mv_dwn(leg_travel_dist/3)
            self.legs[3].fd_mv_dwn(leg_travel_dist/3)
        elif(self.cycle==2):
            self.legs[0].fd_mv_dwn()
            self.legs[1].fd_mv_dwn(leg_travel_dist/3)
            self.legs[2].fd_mv_up(5*leg_travel_dist/6,(0.5*leg_travel_dist/6,robo_height-self.legs[2].l3))
            self.legs[3].fd_mv_dwn(leg_travel_dist/3)

class custom_dim():
    def __init__(self, label=None, size=0, stride=0):
        self.label = label
        self.size = size
        self.stride = stride

robo_height = rospy.get_param('robo_height',7.5)
leg1=legjoints('hind',1)
leg2=legjoints('front',2)
leg3=legjoints('hind',3)
leg4=legjoints('front',4)
joint_states=JointState()
js_real=Float64MultiArray()
rospy.init_node('joint_state_publisher')
# pub = rospy.Publisher('joint_states', JointState, queue_size=2)
pub_real=rospy.Publisher('ele_position_controller/command',Float64MultiArray,queue_size=2)
rate = rospy.Rate(30) # 10hz

c = custom_dim('', 12, 1)
js_real.layout.dim =[c]
js_real.layout.data_offset=0


def talker():
    global leg1,leg2,leg3,leg4,joint_states,num_of_pt,pub,rate,robo_height,js
    legs=move_fns(leg1,leg2,leg3,leg4)

    while not rospy.is_shutdown():  
        joint_states.name.clear()
        joint_states.position.clear()
        js_real.data.clear()
        joint_states.header.stamp=rospy.get_rostime()
        h=rospy.get_param('/robo_height')
        if(h!=robo_height):
            legs.sns(h)   
            robo_height=h
        legs.gait('amble')
        # legs.stop=True
        # legs.turn('right')
        # leg1.fd_mv_dwn('front')
        for i in range(12):
            js_real.data.append(joint_states.position[i])
        # print(js_real.data)

        # print(joint_states)
        # pub.publish(joint_states)
        pub_real.publish(js_real)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

import math
import time

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

print(point_finder('linear',(0,0),5,10,180))
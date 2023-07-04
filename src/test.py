import math
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

print(point_finder('circle',(1,1),2,5))

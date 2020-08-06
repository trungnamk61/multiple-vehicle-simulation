import cv2
import numpy as np 
import time
import copy
from numpy import random
from threading import Thread
import threading

class KalmanFilter:
    def __init__(self) :
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        # kf.statePre = np.array([[401], [588]])
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        self.measured = np.array([[np.float32(coordX-initState[0])], [np.float32(coordY-initState[1])]])
        correct = self.kf.correct(self.measured)
        predicted = self.kf.predict()
        predicted[0],predicted[1] = predicted[0]+initState[0],predicted[1]+initState[1]
        return predicted


# Define a vehicle with line trajectory
class Trajectory:
    'linear equations: y=ax+b '
    def __init__(self, startx, a, b, vehicletype:'car', color:'red', speedStart: int,accel: int,weight: int,identify : int):
        self.startx=startx
        self.a=a
        self.b=b
        self.vehicletype=vehicletype
        self.color=color
        self.speedStart=speedStart
        self.accel=accel
        self.weight = weight
        self.speed = 0
        self.identify = identify
        # Toa do vehicle trong khung hinh
        self.current_coordinate = (int(startx),int(a*startx+b))
        (self.initState_x,self.initState_y) = self.current_coordinate
        # Location he quy chieu mat dat
        self.location_x, self.location_y = [self.current_coordinate[0]-640, self.current_coordinate[1]-360]
        self.trajectory = [self.location_x,self.location_y]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def update(self):
        # Update position vehicle in frame
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x + self.speed
        new_y = cur_y + self.a*self.speed
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate=new_coordinate
        # Update location and add to trajectory
        cur_lx, cur_ly = self.location_x,self.location_y
        new_lx = cur_lx + self.speed
        new_ly = self.a*new_lx +self.b
        self.location_x,self.location_y = [int(new_lx),int(new_ly)]
        location = [int(new_lx),int(new_ly)]
        # print(self.location)
        # Update trajectory
        self.trajectory.append(location)
        return self.location_x,self.location_y
        # print(self.trajectory)
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        self.measured = np.array([[np.float32(coordX-self.initState_x)], [np.float32(coordY-self.initState_y)]])
        correct = self.kf.correct(self.measured)
        predicted = self.kf.predict()
        predicted[0],predicted[1]=predicted[0]+self.initState_x,predicted[1]+self.initState_y
        predicted = predicted.astype(np.int)
        return (predicted[0],predicted[1])

    def UAVmoveForward(self, UAVspeedx):
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x - UAVspeedx
        new_y = cur_y
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate = new_coordinate

    def UAVturnRight(self, UAVspeed):
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x
        new_y = cur_y - UAVspeed
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate = new_coordinate
    def __delete__(self) : 
        print('deleted'+self.vehicletype+self.color)

# Set parameters for UAV
# UAV_speed = 15
# Altitude
# UAV_start
# UAV_current
# UAV_trajectory



class uav_class:
    def __init__(self):
        #Khoi tao toa do UAV
        self.location = [0, 0]
        #Van toc
        self.speed = 0
        #Speed per frame pixel/frame
        self.speed_int = 0
        #Gia toc
        self.a = 0
        #Do cao
        self.h = 70
        self.speedMax = 200
    def calculate_acceleration(self):
        return 0

    #Update speed per frame
    def update_speed(self, t = 1):
        # V = Vo + at 
        new_speed = self.speed + self.a*t
        self.speed = new_speed
        self.speed_int = round(self.speed)

# def rejectObj(vehicle: list,): 


#Output video
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
output_movie = cv2.VideoWriter('simulation.mp4', fourcc, 20, (1280, 720))

# Set background
background=cv2.imread('images/ground1.jpg',1)

# Initialization vehicles
vehicles =[]
vehicles.append( Trajectory(500, 0, 350, 'car', 'yellow', 20,0.0013,random.randint(1,10),0) )
vehicles.append( Trajectory(700, 0, 320, 'car', 'red', 10,0.0011,random.randint(1,10),1) )
vehicles.append( Trajectory(250, 0, 220, 'car', 'green', 11,0.001,random.randint(1,10),2) )
vehicles.append( Trajectory(200, 0, 350, 'car', 'blue', 11,0.0012,random.randint(1,10),3) )
vehicles.append( Trajectory(-100, 0, 450, 'car', 'pink',12,0.0011,random.randint(1,10),4))
vehicles.append( Trajectory(-50, 0, 300, 'car', 'pink',10,0,0,5) )          
vehicles.append( Trajectory(1000, 0, 450, 'car', 'green',10,0,0,6) )
vehicles.append( Trajectory(1700, 0, 300, 'car', 'yellow',10,0,0,7) )
# vehicles.append( Trajectory(2300, 0, 500, 'truck', 'green',12,0,0,8) )
vehicles.append( Trajectory(2800, 0, 270, 'car', 'orange',10,0,0,9) )
vehicles.append( Trajectory(3500, 0, 400, 'car', 'white',10,0,0,10) )
# vehicles.append( Trajectory(4000, 0, 500, 'truck', 'yellow',12,0,0,11) )
# vehicles.append( Trajectory(4500, 0, 450, 'truck', 'orange',10,0,0,12) )
# vehicles.append( Trajectory(5000, 0, 470, 'truck', 'red',10,0.015,0,14) )
# vehicles.append( Trajectory(5500, 0, 450, 'truck', 'orange',10,0,0,14) )
vehicles.append( Trajectory(4000, 0, 350, 'car', 'orange',10,0.005,0,11) )
vehicles.append( Trajectory(4500, 0, 250, 'car', 'blue',10,0.005,0,12) )
vehicles.append( Trajectory(5500, 0, 300, 'car', 'yellow',10,0.005,0,13) )
vehicles.append( Trajectory(7000, 0, 300, 'car', 'red',10,0,0,14) )
vehicles.append( Trajectory(8500, 0, 290, 'car', 'blue',10,0,0,15) )
# vehicles.append( Trajectory(5000, 0, 250, 'car', 'blue',2,0,0,19) )
vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,16) )
# vehicles.append( Trajectory(5000, 0, 450, 'car', 'blue',2,0,0,21) )
# vehicles.append( Trajectory(5000, 0, 550, 'car', 'blue',2,0,0,22) )
# vehicles.append( Trajectory(950, 0, 350, 'car', 'red',10,0.005,0) )
list_vehicle = [vehicles[0], vehicles[1], vehicles[2],vehicles[3],vehicles[4]]
list_must_vehicle = [vehicles[0], vehicles[1], vehicles[2],vehicles[3],vehicles[4]]
list_vehicle_identify = [0,1,2,3,4] 
list_vehicle_coordinates = []
list_vehicle_predict = []
print((list_vehicle))
numObj = 4
uav = uav_class()
count = 0
locationx = 0
locationy = 0
UAV_speed_x = 0
UAV_speed_y = 0
num_max = 0
num_min = 0
num_reject = 0
list_reject = []
controll = 0


#Draw vehicle on image
def drawVehicle(image: np.zeros((720,1280,3), np.uint8), center_coordinates, vehicletype, color):
    vehicle_icon = cv2.imread('images/' + vehicletype + '-' + color + '.jpg',1)
    height_vehicle, width_vehicle = vehicle_icon.shape[0:2]
    delta_x = int((width_vehicle - 1)/2)
    delta_y = int((height_vehicle -1)/2)
    # print(delta_x, delta_y)
    # print('SHAPE: ',vehicle_icon.shape)
    # Xe nam tron trong khung hinh
    if center_coordinates[0] >= delta_x and center_coordinates[1] >= delta_y and 1280 - center_coordinates[0] > delta_x and 720 - center_coordinates[1] > delta_y:
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, 
            center_coordinates[0] - delta_x: center_coordinates[0] + delta_x +1] = vehicle_icon
    
    # Xe tran trai khung hinh
    if center_coordinates[0] < delta_x and center_coordinates[0] + delta_x > 0 :
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, 0:center_coordinates[0] + delta_x + 1] = vehicle_icon[:,width_vehicle - center_coordinates[0]-delta_x - 1:width_vehicle]
    # Xe tran phai khung hinh
    if center_coordinates[0] > 1280 - delta_x and center_coordinates[0]-delta_x < 1280:
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, center_coordinates[0] - delta_x:1280] = vehicle_icon[:,0:1280 - center_coordinates[0] + delta_x]
    # Xe tran phai khung hinh
    # if center_coordinates
    # print('center_coordinates : ',[center_coordinates[0] ,center_coordinates[1] ])
    return image

def drawMotionVector(image, source, destination):
    # Green color in BGR  
    color = (255, 255, 0)  
    
    # Line thickness of 9 px  
    thickness = 5
    
    # Using cv2.arrowedLine() method  
    # Draw a diagonal green arrow line 
    # with thickness of 9 px  
    image = cv2.arrowedLine(image, source, destination, color, thickness)
    return image


def drawPointVehicle(image: np.zeros((720,1280,3), np.uint8), _center_coordinates, color):

    # Radius of circle 
    radius = 7
    
    # Line thickness of -1 px 
    thickness = -1
    
    # Using cv2.circle() method 
    # Draw a circle of red color of thickness -1 px 
    image = cv2.circle(image, _center_coordinates, radius, color, thickness) 
    return image

def drawROI(image, start, end, color = (0, 255, 255)):

    # Line thickness of 2 px 
    thickness = 2
    
    # Using cv2.rectangle() method 
    # Draw a rectangle with blue line borders of thickness of 2 px 
    image = cv2.rectangle(image, start, end, color, thickness) 
    return image

# Find region of interest - bouding box
def findROI(list_vehicle_coordinates:list):
    start_point = ()
    end_point = ()
    x_min = 960
    x_max = 0
    y_min = 640
    y_max = 0
    i=0
    numObj_max = 0
    numObj_min = 0
    for vehicle_coordinate in list_vehicle:
        if x_max < vehicle_coordinate.current_coordinate[0]:
            x_max = vehicle_coordinate.current_coordinate[0]
            numObj_max = vehicle_coordinate.identify
        if x_min > vehicle_coordinate.current_coordinate[0]:
            x_min = vehicle_coordinate.current_coordinate[0]
            numObj_min = vehicle_coordinate.identify
        if y_max < vehicle_coordinate.current_coordinate[1]:
            y_max = vehicle_coordinate.current_coordinate[1]   
        if y_min> vehicle_coordinate.current_coordinate[1]:
            y_min = vehicle_coordinate.current_coordinate[1]
        start_point = (x_min, y_min)
        end_point = (x_max, y_max)
    return (start_point, end_point),numObj_max,numObj_min

    

def drawUAVInfo(image, speed, location, altitude, _time, num_tracking=4, num_missing=0):
    black_block = np.zeros((35,1280,3), np.uint8)
    image[0:35, 0:1280] = black_block
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (255, 255, 255)
    image = cv2.putText(image, 'Speed: '+str(speed), (80,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Location: '+str(location), (280,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Altitude: '+str(altitude), (540,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Time: '+str(_time), (720,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Num_Tracking: '+str(num_tracking), (860,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Num_Missing: '+str(num_missing), (1080,22), font , 0.6, color, 1, cv2.LINE_AA)
    return image
def updateVehicle (controll:int,num:int,location_x_uav : int) :
    num_comeback = -1
    list_vehicle_coordinates = []
    # print("list vehicles",len(list_vehicle))
    if (controll != 0 ):
        #dreject object tracking
        list_reject.append(num)
        print("list reject",list_reject)
        # for i,vehicle in enumerate(list_vehicle):
        #     if vehicle.identify==num :
        #         print(vehicle.identify)
        for j,iden in enumerate(list_vehicle_identify) :
            if (iden==num) : 
                del list_vehicle_identify[j]
                del list_vehicle[j]
        num = 0       
        controll = 0
    a = len(list_reject)
    for i in range(a) :
        # print(vehicles[list_reject[i]].current_coordinate[0],vehicles[list_reject[i]].current_coordinate[1])
        if abs(vehicles[list_reject[i]].location_x - location_x_uav) < 640  :
            print("xxxxxxxx")
            print("comback")
            num_comeback = list_reject[i]
            del list_reject[i]
            break
        # print(type(list_vehicle_coordinates))
    if ( num_comeback>=0 ) : 
        # print("aa")
        for xi in range(len(vehicles)):
            if xi==num_comeback :
                # print("i'm comback")
                list_vehicle_identify.append(xi)
                list_vehicle.append(vehicles[xi])
                # print(len(list_vehicle))   

    for vehicle in list_vehicle :
        list_vehicle_coordinates.append((vehicle.current_coordinate))
    return (list_vehicle_coordinates),num,controll
def updateVehicle_predict(controll:int ):
    list_vehicle_predict = []
    for vehicle in list_vehicle :
        list_vehicle_predict.append(vehicle.Estimate(
                vehicle.current_coordinate[0],vehicle.current_coordinate[1]
                )
        )
    # print(list_vehicle_predict)
    return (list_vehicle_predict)


def avoid_vehicle() :
    slow = 0
    right =0 
    left = 0
    x = 0
    y = 0
    thress = 0
    j = 0
    left,right,slow,thress = 0,0,0,0
    x,y = 0,0
    for j in range(len(list_vehicle)) :
        # if (vehicles[j].current_coordinate[1] > 300 
        #     and vehicles[j].current_coordinate[1] < 500) :
          for i in range(len(vehicles)) :
            left,right,slow,thress = 0,0,0,0
            x,y = 0,0
            if i!= j :
                k,h = 0,0
                if (vehicles[j].current_coordinate[0] >= vehicles[i].current_coordinate[0]  - 250 
                        and vehicles[j].current_coordinate[0]  <= vehicles[i].current_coordinate[0] 
                        and vehicles[j].current_coordinate[1]  >= vehicles[i].current_coordinate[1] - 60 
                        and vehicles[j].current_coordinate[1]  <= vehicles[i].current_coordinate[1]  ):
                    # print ("aaaa")
                    for k in range(len(vehicles)) :
                        if k != j and k != i: 
                            if (vehicles[k].current_coordinate[0] <= vehicles[i].current_coordinate[0]
                                    and vehicles[k].current_coordinate[0] >= vehicles[i].current_coordinate[0]-266 
                                    and vehicles[k].current_coordinate[1] <= vehicles[i].current_coordinate[1] 
                                    and  vehicles[k].current_coordinate[1] >= vehicles[i].current_coordinate[1]-130) :
                                x = 1
                                for h in range(len(vehicles)) :
                                    if h != k and h != i and h!=j : 
                                        if (vehicles[h].current_coordinate[0] <= vehicles[i].current_coordinate[0] 
                                                and vehicles[h].current_coordinate[0] >= vehicles[i].current_coordinate[0] - 266 
                                                and vehicles[h].current_coordinate[1] >= vehicles[i].current_coordinate[1]  
                                                and  vehicles[h].current_coordinate[1] <= vehicles[i].current_coordinate[1] + 130) :
                                            x = 2
                                                    
                    # if (vehicles[j].current_coordinate[1] < 300 and x==0 ):
                    #     right = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and x==0 ):
                    #     left = 1     
                    # if (vehicles[j].current_coordinate[1] < 300 and x==1 ) :
                    #     slow = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and x==1 ) :
                    #     slow = 1
                    if (vehicles[j].current_coordinate[1]  >= 200 and vehicles[j].current_coordinate[1] <=600) :              
                        if (x == 2) :
                            slow = 1
                        if (x ==1 ) :
                            right = 1
                        if (x == 0 ) :
                            left = 1
                if (vehicles[j].current_coordinate[0]  > (vehicles[i].current_coordinate[0] -250) 
                        and vehicles[j].current_coordinate[0]  < vehicles[i].current_coordinate[0] 
                        and vehicles[j].current_coordinate[1]  > vehicles[i].current_coordinate[1]  
                        and vehicles[j].current_coordinate[1]  < vehicles[i].current_coordinate[1] + 60 ):
                    for h in range(len(vehicles)) :
                        if h != j and h != i: 
                            if (vehicles[h].current_coordinate[0] < vehicles[i].current_coordinate[0] 
                                    and vehicles[h].current_coordinate[0] > vehicles[i].current_coordinate[0]-266
                                    and vehicles[h].current_coordinate[1] > vehicles[i].current_coordinate[1]  
                                    and  vehicles[h].current_coordinate[1] < vehicles[i].current_coordinate[1] + 130) :
                                # print ("aaaa")
                                y = 1 
                                # for k in range(len(vehicles)) :
                                #     if k != h and k != i and k!=j : 
                                #         if (vehicles[k].current_coordinate[0] < vehicles[i].current_coordinate[0] 
                                #                 and vehicles[k].current_coordinate[0] > vehicles[i].current_coordinate[0]-266
                                #                 and vehicles[k].current_coordinate[1] > vehicles[i].current_coordinate[1] 
                                #                 and  vehicles[k].current_coordinate[1] < vehicles[i].current_coordinate[1] -130) :
                                #             y = 2
                                                    
                    # if (vehicles[j].current_coordinate[1] < 300 and y==0 ):
                    #     right = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and y==0 ):
                    #     left = 1     
                    # if (vehicles[j].current_coordinate[1] < 300 and y==1 ) :
                    #     slow = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and y==1 ) :
                    #     slow = 1
                    if (vehicles[j].current_coordinate[1]  >= 200 and vehicles[j].current_coordinate[1]  <=600) :              
                        # if (y == 2) :
                        #     slow = 1
                        if (y ==1 ) :
                            left = 1
                        if (y == 0 ) :
                            right = 1
                        
                if (slow == 1) : 
                    vehicles[j].speedStart = 0
                    # vehicles[j].accel =0
                    # print("xe ",j)
                    # print("slow")
                if (right == 1) :
                    vehicles[j].current_coordinate = (vehicles[j].current_coordinate[0]),(vehicles[j].current_coordinate[1]+5)
                    # print("xe ",j)
                    # print("right")
                if (left == 1) :
                    # print("xe ",j)
                    # print("left")
                    vehicles[j].current_coordinate = (vehicles[j].current_coordinate[0]),(vehicles[j].current_coordinate[1]-5)



#Running
while(1):
    count+=1
    #list_vehicle_tracking initially
    list_vehicle_coordinates,num_reject,controll =updateVehicle(controll,num_reject,locationx)
    ###### Animation UAV Flight ###########  BLOCK_START ############
    if UAV_speed_x >= 0:
    # UAV move forward
        new_background = np.zeros((720,1280,3), dtype=np.uint8)
        new_background[0:720, 0:(1280-UAV_speed_x)] = background[0:720, UAV_speed_x:1280]
        new_background[0:720, (1280-UAV_speed_x):1280] = background[0:720, 0:UAV_speed_x]
        background = new_background
    else:
        # UAv move backward
        UAV_speed_x = - UAV_speed_x
        new_background = np.zeros((720,1280,3), dtype=np.uint8)
        new_background[0:720, UAV_speed_x:1280] = background[0:720, 0:(1280-UAV_speed_x)]
        new_background[0:720, 0:UAV_speed_x] = background[0:720, (1280-UAV_speed_x):1280]
        background = new_background
        UAV_speed_x = -UAV_speed_x
    if UAV_speed_y >=0:
        # UAV turn right
        new_background[0:(720-UAV_speed_y),0:1280] = background[UAV_speed_y:720, 0:1280]
        new_background[(720-UAV_speed_y):720, 0:1280] = background[0:UAV_speed_y, 0:1280]
        background = new_background
    else:
        # UAV turn left
        UAV_speed_y = -UAV_speed_y
        new_background[UAV_speed_y:720,0:1280] = background[0:(720-UAV_speed_y), 0:1280]
        new_background[0:UAV_speed_y, 0:1280] = background[(720-UAV_speed_y):720, 0:1280]
        background = new_background
        UAV_speed_y = -UAV_speed_y

    cur_scene = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene = copy.copy(background)
    ###### Animation UAV Flight ###########  BLOCK_END ############

    print(vehicles[0].location_x)
    #Draw info of UAV
    cur_scene = drawUAVInfo(cur_scene, (UAV_speed_x, UAV_speed_y), (locationx,locationy), 70, count,len(list_vehicle),5-len(list_vehicle))

    avoid_vehicle()
    for vehicle in vehicles:
        # vehicle.weight = random.randint(10)
        cur_scene = drawVehicle(cur_scene, vehicle.current_coordinate, vehicle.vehicletype, vehicle.color)
        # cur_scene = drawPointVehicle(cur_scene, vehicle.speedStart, (0,0,255))
        if vehicle.weight > 0:    
            cur_scene = cv2.putText(cur_scene, str(float(vehicle.weight)), vehicle.current_coordinate, cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
        vehicle.update()
        vehicle.UAVturnRight(UAV_speed_y)
        vehicle.UAVmoveForward(UAV_speed_x)
    ROI,num_max,num_min = findROI(list_vehicle_coordinates)
    # xcenter_ROI = int((ROI[0][0]+ROI[1][0])/2)
    # ycenter_ROI = int((ROI[0][1]+ROI[1][1])/2)
    # center_ROI =(xcenter_ROI, ycenter_ROI)
    drawROI(cur_scene, ROI[0], ROI[1],(255,0,0))
    # cur_scene = cv2.putText(cur_scene, 'CurrentROI', ROI[0], cv2.FONT_HERSHEY_SIMPLEX , 0.6, (0,255,255), 2, cv2.LINE_AA)

    list_vehicle_predict= updateVehicle_predict(controll)
    ROI_predict , num_max , num_min= findROI(list_vehicle_predict)
    # print("num reject ",num_reject)
        # print(vehicles[0].current_coordinate)
    xcenter_ROI_predict = int((ROI_predict[0][0]+ROI_predict[1][0])/2)
    ycenter_ROI_predict = int((ROI_predict[0][1]+ROI_predict[1][1])/2)
    center_ROI =(xcenter_ROI_predict, ycenter_ROI_predict)
    drawROI(cur_scene, ROI_predict[0], ROI_predict[1],(255,255,0))
    if ((ROI_predict[1][0] - ROI_predict[0][0]) > 1200) :
        if (vehicles[num_max].weight > vehicles[num_min].weight) : 
                num_reject = num_min
        else : 
                num_reject = num_max
        controll = 1
    # print(controll)


    # cur_scene = cv2.putText(cur_scene, 'PredictiveROI', ROI_predict[1], cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
    if (count == 200) :
        vehicles[0].speedStart = 0
        vehicles[0].accel = 0
    if (count == 300) :
        vehicles[0].speedStart = 17
        vehicles[0].accel = 0.0013
    if count>10:
        if xcenter_ROI_predict - 640 > -20 and UAV_speed_x <= uav.speedMax:
            UAV_speed_x+=2
        if xcenter_ROI_predict - 640 < 20 and UAV_speed_x >=0:
            UAV_speed_x-=1
        if ycenter_ROI_predict - 360 > -10 and UAV_speed_y <= 1:
            UAV_speed_y+=1
        if ycenter_ROI_predict - 360 < 10 and UAV_speed_y >=-1:
            UAV_speed_y-=1
        for vehicle in vehicles:
            vehicle.speed = 10*vehicle.accel + vehicle.speedStart
            vehicle.speedStart = vehicle.speed
        for vehicle in list_vehicle :
            if (vehicle.speedStart > uav.speedMax) :
                controll = 1
    # if (count == 300 ) : 
    #     vehicles[1].accel == -0.15
    # if (count == 350 ) :
    #     vehicles[0].accel == -0.5
    cv2.imshow('frame',cur_scene)
    output_movie.write(cur_scene)
    locationx = locationx + UAV_speed_x
    locationy = locationy + UAV_speed_y
    print(locationx,locationy)
    time.sleep(0.1)
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
import cv2
import numpy as np 
import time
import copy
from numpy import random
from threading import Thread
import threading
import imutils
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
        self.speed_x = 0
        self.speed_y = 0
        self.identify = identify
        # Toa do vehicle trong khung hinh
        self.current_coordinate = (int(startx),int(a*startx+b))
        (self.initState_x,self.initState_y) = self.current_coordinate
        # Location he quy chieu mat dat
        self.location_x, self.location_y = [self.current_coordinate[0], self.current_coordinate[1]]

        self.trajectory = [self.location_x,self.location_y]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def update(self):
        
        # Update position vehicle in frame
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x + self.speed_x
        new_y = cur_y + self.a*self.speed_x
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate=new_coordinate
        # Update location and add to trajectory
        cur_lx, cur_ly = self.location_x,self.location_y
        new_lx = cur_lx + self.speed_x
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
    def __init__(self,identify : int,locationx_uav :float,locationy_uav :float):
        #Khoi tao toa do UAV
        self.identify = identify
        self.locationx_uav = locationx_uav
        self.locationy_uav = locationy_uav
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
output_movie = cv2.VideoWriter('simulation.mp4', fourcc, 6, (1280, 720))

# Set background
background1=cv2.imread('images/ground1.jpg',1)
background2 = background1
background2 [0:720,0:640] = background1[0:720,640:1280]
background2 [0:720,640:1280] = background1[0:720,0:640]
background3 = background1
background4 = background2
background5 = background1
background6 = background2

partition = cv2.imread('images/partition.png',1)
dsize = (20,720)
partition = cv2.resize(partition,dsize)
# height, width = partition.shape[0:2]
# print(partition.shape[0:2])
# Initialization vehicles
vehicles =[]
#group 1
vehicles.append( Trajectory(900, 0, 350, 'car', 'yellow', 20 ,0.0013,random.randint(1,10),0) )
vehicles.append( Trajectory(700, 0, 320, 'car', 'red', 11,0.0011,random.randint(1,10),1) )
vehicles.append( Trajectory(250, 0, 400, 'car', 'green', 11,0.0011,random.randint(1,10),2) )
vehicles.append( Trajectory(200, 0, 300, 'car', 'blue', 11,0.0011,random.randint(1,10),3) )
vehicles.append( Trajectory(-100, 0, 450, 'car', 'pink',11,0.0011,random.randint(1,10),4))
# vehicles.append( Trajectory(-50, 0, 300, 'car', 'orange',11,0.0011,random.randint(1,10),5) )     
# vehicles.append( Trajectory(600, 0, 400, 'car', 'red',11,0.0011,random.randint(1,10),6) )  
# vehicles.append( Trajectory(500, 0, 220, 'car', 'yellow',11,0.0011,random.randint(1,10),7) )     
# vehicles.append( Trajectory(-40, 0, 450, 'car', 'blue',11,0.0011,random.randint(1,10),8) )     
# vehicles.append( Trajectory(600, 0, 500, 'car', 'green',11,0.0011,random.randint(1,10),9) )       

#group 2

vehicles.append( Trajectory(1940, 0, 220, 'car', 'blue',11,0.0011,random.randint(1,10),5) )       
vehicles.append( Trajectory(2200, 0, 380, 'car', 'red',11,0.0011,random.randint(1,10),6) )       
vehicles.append( Trajectory(2400, 0, 470, 'car', 'green',11,0.0011,random.randint(1,10),7) )       
vehicles.append( Trajectory(2620, 0, 250, 'car', 'orange',11,0.0011,random.randint(1,10),8) )       
vehicles.append( Trajectory(2800, 0, 300, 'car', 'yellow',11,0.0011,random.randint(1,10),9) )       
# vehicles.append( Trajectory(2600, 0, 350, 'car', 'red',11,0.0011,random.randint(1,10),15) )  
# vehicles.append( Trajectory(2700, 0, 520, 'car', 'yellow',11,0.0011,random.randint(1,10),16) )  
# vehicles.append( Trajectory(2900, 0, 350, 'car', 'green',11,0.0011,random.randint(1,10),17) )  
# vehicles.append( Trajectory(2800, 0, 220, 'car', 'blue',11,0.0011,random.randint(1,10),18) )  
# vehicles.append( Trajectory(3000, 0, 500, 'car', 'orange',11,0.0011,random.randint(1,10),19) )  

#group 3
vehicles.append( Trajectory(3900, 0, 330, 'car', 'yellow',11,0.0013,random.randint(1,10),10) )       
vehicles.append( Trajectory(4100, 0, 200, 'car', 'blue',11,0.0013,random.randint(1,10),11) )       
vehicles.append( Trajectory(4350, 0, 500, 'car', 'red',11,0.0013,random.randint(1,10),12) )       
vehicles.append( Trajectory(4670, 0, 420, 'car', 'orange',11,0.0013,random.randint(1,10),13) )       
vehicles.append( Trajectory(4800, 0, 300, 'car', 'blue',11,0.0013,random.randint(1,10),14) )       
# vehicles.append( Trajectory(4400, 0, 370, 'car', 'yellow',11,0.0013,random.randint(1,10),25) )    
# vehicles.append( Trajectory(4620, 0, 470, 'car', 'red',11,0.0013,random.randint(1,10),26) )  
# vehicles.append( Trajectory(4700, 0, 270, 'car', 'green',11,0.0013,random.randint(1,10),27) )  
# vehicles.append( Trajectory(4890, 0, 370, 'car', 'yellow',11,0.0013,random.randint(1,10),28) )     
# vehicles.append( Trajectory(5020, 0, 200, 'car', 'blue',11,0.0013,random.randint(1,10),29) )     

#group 4
vehicles.append( Trajectory(5800, 0, 500, 'car', 'blue',11,0.0013,random.randint(1,10),15) )       
vehicles.append( Trajectory(5970, 0, 220, 'car', 'green',11,0.0013,random.randint(1,10),16) )       
vehicles.append( Trajectory(6170, 0, 370, 'car', 'red',11,0.0013,random.randint(1,10),17) )       
vehicles.append( Trajectory(6450, 0, 400, 'car', 'yellow',11,0.0013,random.randint(1,10),18) )       
vehicles.append( Trajectory(6770, 0, 500, 'car', 'green',11,0.0013,random.randint(1,10),19) )       
# vehicles.append( Trajectory(6450, 0, 280, 'car', 'blue',11,0.0013,random.randint(1,10),35) ) 
# vehicles.append( Trajectory(6710, 0, 450, 'car', 'green',11,0.0013,random.randint(1,10),36) )  
# vehicles.append( Trajectory(6850, 0, 320, 'car', 'red',11,0.0013,random.randint(1,10),37) )  
# vehicles.append( Trajectory(6900, 0, 200, 'car', 'red',11,0.0013,random.randint(1,10),38) )        
# vehicles.append( Trajectory(7000, 0, 350, 'car', 'blue',11,0.0013,random.randint(1,10),39) )        

#group 5
vehicles.append( Trajectory(7700, 0, 450, 'car', 'red',11,0.0013,random.randint(1,10),20) )       
vehicles.append( Trajectory(7980, 0, 300, 'car', 'green',11,0.0013,random.randint(1,10),21) )       
vehicles.append( Trajectory(8150, 0, 200, 'car', 'blue',11,0.0013,random.randint(1,10),22) )       
vehicles.append( Trajectory(8470, 0, 250, 'car', 'green',11,0.0013,random.randint(1,10),23) )       
vehicles.append( Trajectory(8700, 0, 400, 'car', 'green',11,0.0013,random.randint(1,10),24) )       
# vehicles.append( Trajectory(8200, 0, 500, 'car', 'red',11,0.0013,random.randint(1,10),45) )  
# vehicles.append( Trajectory(8340, 0, 450, 'car', 'yellow',11,0.0013,random.randint(1,10),46) )  
# vehicles.append( Trajectory(8610, 0, 450, 'car', 'blue',11,0.0013,random.randint(1,10),47) )  
# vehicles.append( Trajectory(8720, 0, 350, 'car', 'red',11,0.0013,random.randint(1,10),48) )       
# vehicles.append( Trajectory(8800, 0, 250, 'car', 'yellow',11,0.0013,random.randint(1,10),49) )       

#group 6
vehicles.append( Trajectory(9650, 0, 200, 'car', 'red',29,0.0013,random.randint(1,10),25) )       
vehicles.append( Trajectory(9900, 0, 300, 'car', 'green',29,0.0013,random.randint(1,10),26) )       
vehicles.append( Trajectory(10150, 0, 500, 'car', 'blue',29,0.0013,random.randint(1,10),27) )       
vehicles.append( Trajectory(10450, 0, 250, 'car', 'yellow',29,0.0013,random.randint(1,10),28) )       
vehicles.append( Trajectory(10650, 0, 200, 'car', 'red',29,0.0013,random.randint(1,10),29) )       
# vehicles.append( Trajectory(10100, 0, 370, 'car', 'green',29,0.0013,random.randint(1,10),55) ) 
# vehicles.append( Trajectory(10220, 0, 450, 'car', 'yellow',29,0.0013,random.randint(1,10),56) )  
# vehicles.append( Trajectory(10400, 0, 300, 'car', 'yellow',29,0.0013,random.randint(1,10),57) )  
# vehicles.append( Trajectory(10650, 0, 500, 'car', 'green',29,0.0013,random.randint(1,10),58) )    
# vehicles.append( Trajectory(10780, 0, 250, 'car', 'red',29,0.0013,random.randint(1,10),59) )       


#group object ubtracking 
# vehicles.append( Trajectory(900, 0, 480, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(400, 0, 250, 'car', 'red',10,0,0,19) )
# vehicles.append( Trajectory(600, 0, 350, 'car', 'green',10,0,0,19) )
# vehicles.append( Trajectory(50, 0, 210, 'car', 'yellow',10,0,0,19) )
# vehicles.append( Trajectory(1000, 0, 330, 'car', 'orange',10,0,0,19) )
# vehicles.append( Trajectory(100, 0, 470, 'car', 'red',10,0,0,19) )


# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )

# vehicles.append( Trajectory(1000, 0, 450, 'car', 'green',10,0,0,10) )
# vehicles.append( Trajectory(1700, 0, 300, 'car', 'yellow',10,0,0,11) )
# vehicles.append( Trajectory(2800, 0, 270, 'car', 'orange',10,0,0,12) )
# vehicles.append( Trajectory(3500, 0, 400, 'car', 'white',10,0,0,13) )
# vehicles.append( Trajectory(4000, 0, 350, 'car', 'orange',10,0.005,0,14) )
# vehicles.append( Trajectory(4500, 0, 250, 'car', 'blue',10,0.005,0,15) )
# vehicles.append( Trajectory(5500, 0, 300, 'car', 'yellow',10,0.005,0,16) )
# vehicles.append( Trajectory(7000, 0, 300, 'car', 'red',10,0,0,17) )
# vehicles.append( Trajectory(8500, 0, 290, 'car', 'blue',10,0,0,18) )
# vehicles.append( Trajectory(9000, 0, 400, 'car', 'blue',10,0,0,19) )

uav = []
uav.append(uav_class(1,640,360))
uav.append(uav_class(2,2560,360))
uav.append(uav_class(3,4480,360))
uav.append(uav_class(4,6400,360))
uav.append(uav_class(5,8320,360))
uav.append(uav_class(6,10240,360))

list_vehicle1 = [vehicles[0], vehicles[1], vehicles[2],vehicles[3],vehicles[4]]  
list_vehicle2 = [vehicles[5],vehicles[6],vehicles[7],vehicles[8],vehicles[9]] 
list_vehicle3 = [vehicles[10],vehicles[11],vehicles[12],vehicles[13],vehicles[14]] 
list_vehicle4 = [vehicles[15],vehicles[16],vehicles[17],vehicles[18],vehicles[19]]
list_vehicle5 = [vehicles[20],vehicles[21],vehicles[22],vehicles[23],vehicles[24]]
list_vehicle6 = [vehicles[25],vehicles[26],vehicles[27],vehicles[28],vehicles[29]]

list_must_vehicle1 = [vehicles[0], vehicles[1], vehicles[2],vehicles[3],vehicles[4]] 
list_must_vehicle2 = [vehicles[5],vehicles[6],vehicles[7],vehicles[8],vehicles[9]]
list_must_vehicle3 =[vehicles[10],vehicles[11],vehicles[12],vehicles[13],vehicles[14]] 
list_must_vehicle4 = [vehicles[15],vehicles[16],vehicles[17],vehicles[18],vehicles[19]]
list_must_vehicle5 = [vehicles[20],vehicles[21],vehicles[22],vehicles[23],vehicles[24]]
list_must_vehicle6 = [vehicles[25],vehicles[26],vehicles[27],vehicles[28],vehicles[29]]

list_vehicle_identify1 = [0,1,2,3,4]
list_vehicle_identify2 = [5,6,7,8,9] 
list_vehicle_identify3 = [10,11,12,13,14]
list_vehicle_identify4 = [15,16,17,18,19]
list_vehicle_identify5 = [20,21,22,23,24]
list_vehicle_identify6 = [25,26,27,28,29]

list_vehicle_coordinates1 = []
list_vehicle_coordinates2 = []
list_vehicle_coordinates3 = []
list_vehicle_coordinates4 = []
list_vehicle_coordinates5 = []
list_vehicle_coordinates6 = []

list_vehicle_predict1 = []
list_vehicle_predict2 = []
list_vehicle_predict3 = []
list_vehicle_predict4 = []
list_vehicle_predict5 = []
list_vehicle_predict6 = []

count = 0

numObj_1 = 5
UAV_speed_x_1 = 0
UAV_speed_y_1 = 0
num_max_1 = 0
num_min_1 = 0
num_reject_1 = 0
list_reject_1 = []
controll_1 = 0
p_1 = [1,1,1,1,1]
utility1 = list_vehicle1[0].weight+list_vehicle1[1].weight+list_vehicle1[2].weight+list_vehicle1[3].weight+list_vehicle1[4].weight

numObj_2 = 5
UAV_speed_x_2 = 0
UAV_speed_y_2 = 0
num_max_2 = 0
num_min_2 = 0
num_reject_2 = 0
list_reject_2 = []
controll_2 = 0
p_2 = [0,0,0,0,0,1,1,1,1,1]
utility2 = list_vehicle2[0].weight+list_vehicle2[1].weight+list_vehicle2[2].weight+list_vehicle2[3].weight+list_vehicle2[4].weight

numObj_3 = 5
UAV_speed_x_3 = 0
UAV_speed_y_3 = 0
num_max_3 = 0
num_min_3 = 0
num_reject_3 = 0
list_reject_3 = []
controll_3 = 0
p_3 = [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1]
utility3 = list_vehicle3[0].weight+list_vehicle3[1].weight+list_vehicle3[2].weight+list_vehicle3[3].weight+list_vehicle3[4].weight

numObj_4 = 5
UAV_speed_x_4 = 0
UAV_speed_y_4 = 0
num_max_4 = 0
num_min_4 = 0
num_reject_4 = 0
list_reject_4 = []
controll_4 = 0
p_4 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1]
utility4 = list_vehicle4[0].weight+list_vehicle4[1].weight+list_vehicle4[2].weight+list_vehicle4[3].weight+list_vehicle4[4].weight

numObj_5 = 5
UAV_speed_x_5 = 0
UAV_speed_y_5 = 0
num_max_5 = 0
num_min_5 = 0
num_reject_5 = 0
list_reject_5 = []
controll_5 = 0
p_5 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1]
utility5 = list_vehicle5[0].weight+list_vehicle5[1].weight+list_vehicle5[2].weight+list_vehicle5[3].weight+list_vehicle5[4].weight

numObj_6 = 5
UAV_speed_x_6 = 0
UAV_speed_y_6 = 0
num_max_6 = 0
num_min_6 = 0
num_reject_6 = 0
list_reject_6 = []
controll_6 = 0
p_6 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1]
utility6 = list_vehicle6[0].weight+list_vehicle6[1].weight+list_vehicle6[2].weight+list_vehicle6[3].weight+list_vehicle6[4].weight


list_reject_main = list_reject_1+list_reject_2+list_reject_3+list_reject_4+list_reject_5+list_reject_6
list_vehicle = list_vehicle1+list_vehicle2+list_vehicle3+list_vehicle4+list_vehicle5+list_vehicle6
list_must_vehicle = list_must_vehicle1+list_must_vehicle2+list_must_vehicle3+list_must_vehicle4+list_must_vehicle5+list_must_vehicle6
list_vehicle_identify = list_vehicle_identify1+list_vehicle_identify2+list_vehicle_identify3+list_vehicle_identify4+list_vehicle_identify5+list_vehicle_identify6

pid_last_UAV_1_d_x_error = 0
pid_last_UAV_1_d_y_error = 0

pid_last_UAV_2_d_x_error = 0
pid_last_UAV_2_d_y_error = 0

pid_last_UAV_3_d_x_error = 0
pid_last_UAV_3_d_y_error = 0

pid_last_UAV_4_d_x_error = 0
pid_last_UAV_4_d_y_error = 0

pid_last_UAV_5_d_x_error = 0
pid_last_UAV_5_d_y_error = 0

pid_last_UAV_6_d_x_error = 0
pid_last_UAV_6_d_y_error = 0
#Draw vehicle on image
def drawVehicle(image: np.zeros((720,1280,3), np.uint8), center_coordinates_x,center_coordinates_y, vehicletype, color):
    vehicle_icon = cv2.imread('images/' + vehicletype + '-' + color + '.jpg',1)
    # dsize_vehicle = (101,31)
    # vehicle_icon = cv2.resize(vehicle_icon,dsize_vehicle)
    height_vehicle, width_vehicle = vehicle_icon.shape[0:2]
    delta_x = int((width_vehicle - 1)/2)
    delta_y = int((height_vehicle -1)/2)
    # print(delta_x, delta_y)
    # print('SHAPE: ',vehicle_icon.shape)
    # Xe nam tron trong khung hinh
    if center_coordinates_x >= delta_x and center_coordinates_y >= delta_y and 1280 - center_coordinates_x > delta_x and 720 - center_coordinates_y > delta_y:
        image[center_coordinates_y - delta_y: center_coordinates_y + delta_y +1, 
            center_coordinates_x - delta_x: center_coordinates_x + delta_x +1] = vehicle_icon
    
    # Xe tran trai khung hinh
    if center_coordinates_x < delta_x and center_coordinates_x + delta_x > 0 :
        image[center_coordinates_y- delta_y: center_coordinates_y + delta_y +1, 0:center_coordinates_x + delta_x + 1] = vehicle_icon[:,width_vehicle - center_coordinates_x-delta_x - 1:width_vehicle]
    # Xe tran phai khung hinh
    if center_coordinates_x > 1280 - delta_x and center_coordinates_x-delta_x < 1280:
        image[center_coordinates_y - delta_y: center_coordinates_y + delta_y +1, center_coordinates_x - delta_x:1280] = vehicle_icon[:,0:1280 - center_coordinates_x + delta_x]
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
def findROI(list_vehicle_coordinates:list,list_vehicle : list,uav):
    start_point = ()
    end_point = ()
    x_min = 960
    x_max = 0
    y_min = 640
    y_max = 0
    i=0
    numObj_max = 0
    numObj_min = 0
    z = uav.locationx_uav 
    for vehicle_coordinate in list_vehicle:
        if x_max < (vehicle_coordinate.current_coordinate[0]-z+640):
            x_max = vehicle_coordinate.current_coordinate[0] -z+640
            numObj_max = vehicle_coordinate.identify
        if x_min > (vehicle_coordinate.current_coordinate[0]-z+640):
            x_min = vehicle_coordinate.current_coordinate[0] -z+640
            numObj_min = vehicle_coordinate.identify
        if y_max < vehicle_coordinate.current_coordinate[1]:
            y_max = vehicle_coordinate.current_coordinate[1]   
        if y_min> vehicle_coordinate.current_coordinate[1]:
            y_min = vehicle_coordinate.current_coordinate[1]
        start_point = (x_min, y_min)
        end_point = (x_max, y_max)
    return (start_point, end_point),numObj_max,numObj_min

def concat_tile(im_list_2d):
    return cv2.vconcat([cv2.hconcat(im_list_h) for im_list_h in im_list_2d])

def drawUAVInfo(image, speed, location, altitude, _time, num_tracking=4, num_missing=0, utility = 0,identify = 0):
    black_block = np.zeros((50,1280,3), np.uint8)
    image[0:50, 0:1280] = black_block
    font = cv2.FONT_HERSHEY_DUPLEX
    color = (255, 255, 255)
    image = cv2.putText(image, 'UAV: '+str(identify), (5,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Speed: '+str(speed), (80,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Location: '+str(location), (240,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Altitude: '+str(altitude), (470,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Time: '+str(_time), (600,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Num_Tracking: '+str(num_tracking), (710,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Num_Missing: '+str(num_missing), (900,22), font , 0.6, color, 1, cv2.LINE_AA)
    image = cv2.putText(image, 'Utility: '+str(utility), (1080,22), font , 0.6, color, 1, cv2.LINE_AA)
    return image
def updateVehicle (controll:int,num:int,location_x_uav : int,list_vehicle : list,list_reject:list,list_vehicle_identify:list,p:list,uav) :
    num_comeback = -1
    list_vehicle_coordinates = []
    # print("list vehicles",len(list_vehicle))
    if (controll != 0 ):
        #dreject object tracking
        list_reject.append(num)
        p[num] = 0 
        # print("list reject",list_reject)
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
        if abs(vehicles[list_reject[i]].current_coordinate[0] - location_x_uav) < 640  :
            # print("xxxxxxxx")
            # print("comback")
            num_comeback = list_reject[i]
            p[num_comeback] = 1
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
        list_vehicle_coordinates.append(((vehicle.current_coordinate[0]-uav.locationx_uav+640),vehicle.current_coordinate[1]))
    return list_vehicle_coordinates,list_vehicle,list_reject,p,num,controll
def updateVehicle_predict(controll:int ,list_vehicle :list,uav):
    list_vehicle_predict = []
    for vehicle in list_vehicle :
        list_vehicle_predict.append(vehicle.Estimate(
                vehicle.current_coordinate[0]-uav.locationx_uav+640,vehicle.current_coordinate[1]
                )
        )
    # print(list_vehicle_predict)
    return (list_vehicle_predict)

def time_cumulative_utility (num_max : int,num_min :int,list_must_vehicle : list,p : list,const : int) :
    function_num_min = 0
    function_num_max = 0
    utility = 0
    for i in range(len(list_must_vehicle)) :
        if i == num_min :
            function_num_min += p[i+const] * list_must_vehicle[i].weight
        if i == num_max :
            function_num_max += p[i+const] * list_must_vehicle[i].weight
    if (function_num_max > function_num_min) :
        num_reject = num_min
    else :
        num_reject = num_max
    return num_reject

def avoid_vehicle(list_vehicle : list) :
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
                if (vehicles[j].current_coordinate[0] >= vehicles[i].current_coordinate[0]  - 270 
                        and vehicles[j].current_coordinate[0]  <= vehicles[i].current_coordinate[0] 
                        and vehicles[j].current_coordinate[1]  >= vehicles[i].current_coordinate[1] - 70 
                        and vehicles[j].current_coordinate[1]  <= vehicles[i].current_coordinate[1]  ):
                    # print ("aaaa")
                    for k in range(len(vehicles)) :
                        if k != j and k != i: 
                            if (vehicles[k].current_coordinate[0] <= vehicles[i].current_coordinate[0]
                                    and vehicles[k].current_coordinate[0] >= vehicles[i].current_coordinate[0]-270 
                                    and vehicles[k].current_coordinate[1] <= vehicles[i].current_coordinate[1] 
                                    and  vehicles[k].current_coordinate[1] >= vehicles[i].current_coordinate[1]-70) :
                                x = 1
                                for h in range(len(vehicles)) :
                                    if h != k and h != i and h!=j : 
                                        if (vehicles[h].current_coordinate[0] <= vehicles[i].current_coordinate[0] 
                                                and vehicles[h].current_coordinate[0] >= vehicles[i].current_coordinate[0] - 270 
                                                and vehicles[h].current_coordinate[1] >= vehicles[i].current_coordinate[1]  
                                                and  vehicles[h].current_coordinate[1] <= vehicles[i].current_coordinate[1] + 70) :
                                            x = 2
                                                    
                    # if (vehicles[j].current_coordinate[1] < 300 and x==0 ):
                    #     right = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and x==0 ):
                    #     left = 1     
                    # if (vehicles[j].current_coordinate[1] < 300 and x==1 ) :
                    #     slow = 1
                    # if (vehicles[j].current_coordinate[1] > 500 and x==1 ) :
                    #     slow = 1
                    if (vehicles[j].current_coordinate[1]  >= 150 and vehicles[j].current_coordinate[1] <=650) :              
                        if (x == 2) :
                            slow = 1
                        if (x ==1 ) :
                            right = 1
                        if (x == 0 ) :
                            left = 1
                if (vehicles[j].current_coordinate[0]  > (vehicles[i].current_coordinate[0] -270) 
                        and vehicles[j].current_coordinate[0]  < vehicles[i].current_coordinate[0] 
                        and vehicles[j].current_coordinate[1]  > vehicles[i].current_coordinate[1]  
                        and vehicles[j].current_coordinate[1]  < vehicles[i].current_coordinate[1] + 70 ):
                    for h in range(len(vehicles)) :
                        if h != j and h != i: 
                            if (vehicles[h].current_coordinate[0] < vehicles[i].current_coordinate[0] 
                                    and vehicles[h].current_coordinate[0] > vehicles[i].current_coordinate[0]-270
                                    and vehicles[h].current_coordinate[1] > vehicles[i].current_coordinate[1]  
                                    and  vehicles[h].current_coordinate[1] < vehicles[i].current_coordinate[1] + 70) :
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
                    if (vehicles[j].current_coordinate[1]  >= 150 and vehicles[j].current_coordinate[1]  <=650) :              
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
#function of tracking frame in each uav



#Running
while(1):
    start = time.time()
    count+=1
    utility1 = 0
    utility2 = 0
    utility3 = 0
    utility4 = 0
    utility5 = 0
    utility6 = 0
    list_reject_main = list_reject_1+list_reject_2+list_reject_3+list_reject_4+list_reject_5+list_reject_6
    #list_vehicle_tracking initially

    list_vehicle_coordinates1,list_vehicle1,list_reject_1,p_1,num_reject_1,controll_1 =updateVehicle(controll_1,num_reject_1,uav[0].locationx_uav,list_vehicle1,list_reject_1,list_vehicle_identify1,p_1,uav[0])
    list_vehicle_coordinates2,list_vehicle2,list_reject_2,p_2,num_reject_2,controll_2 =updateVehicle(controll_2,num_reject_2,uav[1].locationx_uav,list_vehicle2,list_reject_2,list_vehicle_identify2,p_2,uav[1])
    list_vehicle_coordinates3,list_vehicle3,list_reject_3,p_3,num_reject_3,controll_3 =updateVehicle(controll_3,num_reject_3,uav[2].locationx_uav,list_vehicle3,list_reject_3,list_vehicle_identify3,p_3,uav[2])    
    list_vehicle_coordinates4,list_vehicle4,list_reject_4,p_4,num_reject_4,controll_4 =updateVehicle(controll_4,num_reject_4,uav[3].locationx_uav,list_vehicle4,list_reject_4,list_vehicle_identify4,p_4,uav[3])    
    list_vehicle_coordinates5,list_vehicle5,list_reject_5,p_5,num_reject_5,controll_5 =updateVehicle(controll_5,num_reject_5,uav[4].locationx_uav,list_vehicle5,list_reject_5,list_vehicle_identify5,p_5,uav[4])    
    list_vehicle_coordinates6,list_vehicle6,list_reject_6,p_6,num_reject_6,controll_6 =updateVehicle(controll_6,num_reject_6,uav[5].locationx_uav,list_vehicle6,list_reject_6,list_vehicle_identify6,p_6,uav[5])    

    ###### Animation UAV Flight ###########  BLOCK_START ############
    for i in range(len(list_vehicle1)) :
        utility1 += list_vehicle1[i].weight  
    for i in range(len(list_vehicle2)) :
        utility2 += list_vehicle2[i].weight 
    for i in range(len(list_vehicle3)) :
        utility3 += list_vehicle3[i].weight 
    for i in range(len(list_vehicle4)) :
        utility4 += list_vehicle4[i].weight 
    for i in range(len(list_vehicle5)) :
        utility5 += list_vehicle5[i].weight 
    for i in range(len(list_vehicle6)) :
        utility6 += list_vehicle6[i].weight 


    if UAV_speed_x_1 >= 0:
    # UAV move forward
        new_background1 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background1[0:720, 0:(1280-UAV_speed_x_1)] = background1[0:720, UAV_speed_x_1:1280]
        new_background1[0:720, (1280-UAV_speed_x_1):1280] = background1[0:720, 0:UAV_speed_x_1]
        background1 = new_background1
    else:
        # UAv move backward
        UAV_speed_x_1 = - UAV_speed_x_1
        new_background1 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background1[0:720, UAV_speed_x_1:1280] = background1[0:720, 0:(1280-UAV_speed_x_1)]
        new_background1[0:720, 0:UAV_speed_x_1] = background1[0:720, (1280-UAV_speed_x_1):1280]
        background1 = new_background1
        UAV_speed_x_1 = -UAV_speed_x_1
    if UAV_speed_y_1 >=0:
        # UAV turn right
        new_background1[0:(720-UAV_speed_y_1),0:1280] = background1[UAV_speed_y_1:720, 0:1280]
        new_background1[(720-UAV_speed_y_1):720, 0:1280] = background1[0:UAV_speed_y_1, 0:1280]
        background1 = new_background1
    else:
        # UAV turn left
        UAV_speed_y_1 = -UAV_speed_y_1
        new_background1[UAV_speed_y_1:720,0:1280] = background1[0:(720-UAV_speed_y_1), 0:1280]
        new_background1[0:UAV_speed_y_1, 0:1280] = background1[(720-UAV_speed_y_1):720, 0:1280]
        background1 = new_background1
        UAV_speed_y_1 = -UAV_speed_y_1


    if UAV_speed_x_2 >= 0:
    # UAV move forward
        new_background2 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background2[0:720, 0:(1280-UAV_speed_x_2)] = background2[0:720, UAV_speed_x_2:1280]
        new_background2[0:720, (1280-UAV_speed_x_2):1280] = background2[0:720, 0:UAV_speed_x_2]
        background2 = new_background2
    else:
        # UAv move backward
        UAV_speed_x_2 = - UAV_speed_x_2
        new_background2 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background2[0:720, UAV_speed_x_2:1280] = background2[0:720, 0:(1280-UAV_speed_x_2)]
        new_background2[0:720, 0:UAV_speed_x_2] = background2[0:720, (1280-UAV_speed_x_2):1280]
        background2 = new_background2
        UAV_speed_x_2 = -UAV_speed_x_2
    if UAV_speed_y_2 >=0:
        # UAV turn right
        new_background2[0:(720-UAV_speed_y_2),0:1280] = background2[UAV_speed_y_2:720, 0:1280]
        new_background2[(720-UAV_speed_y_2):720, 0:1280] = background2[0:UAV_speed_y_2, 0:1280]
        background2 = new_background2
    else:
        # UAV turn left
        UAV_speed_y_2 = -UAV_speed_y_2
        new_background2[UAV_speed_y_2:720,0:1280] = background2[0:(720-UAV_speed_y_2), 0:1280]
        new_background2[0:UAV_speed_y_2, 0:1280] = background2[(720-UAV_speed_y_2):720, 0:1280]
        background2 = new_background2
        UAV_speed_y_2 = -UAV_speed_y_2


    if UAV_speed_x_3 >= 0:
    # UAV move forward
        new_background3 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background3[0:720, 0:(1280-UAV_speed_x_3)] = background3[0:720, UAV_speed_x_3:1280]
        new_background3[0:720, (1280-UAV_speed_x_3):1280] = background3[0:720, 0:UAV_speed_x_3]
        background3 = new_background3
    else:
        # UAv move backward
        UAV_speed_x_3 = - UAV_speed_x_3
        new_background3 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background3[0:720, UAV_speed_x_3:1280] = background3[0:720, 0:(1280-UAV_speed_x_3)]
        new_background3[0:720, 0:UAV_speed_x_3] = background3[0:720, (1280-UAV_speed_x_3):1280]
        background3 = new_background3
        UAV_speed_x_3 = -UAV_speed_x_3
    if UAV_speed_y_3 >=0:
        # UAV turn right
        new_background3[0:(720-UAV_speed_y_3),0:1280] = background3[UAV_speed_y_3:720, 0:1280]
        new_background3[(720-UAV_speed_y_3):720, 0:1280] = background3[0:UAV_speed_y_3, 0:1280]
        background3 = new_background3
    else:
        # UAV turn left
        UAV_speed_y_3 = -UAV_speed_y_3
        new_background3[UAV_speed_y_3:720,0:1280] = background3[0:(720-UAV_speed_y_3), 0:1280]
        new_background3[0:UAV_speed_y_3, 0:1280] = background3[(720-UAV_speed_y_3):720, 0:1280]
        background3 = new_background3
        UAV_speed_y_3 = -UAV_speed_y_3


    if UAV_speed_x_4 >= 0:
    # UAV move forward
        new_background4 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background4[0:720, 0:(1280-UAV_speed_x_4)] = background4[0:720, UAV_speed_x_4:1280]
        new_background4[0:720, (1280-UAV_speed_x_4):1280] = background4[0:720, 0:UAV_speed_x_4]
        background4 = new_background4
    else:
        # UAv move backward
        UAV_speed_x_4 = - UAV_speed_x_4
        new_background4 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background4[0:720, UAV_speed_x_4:1280] = background4[0:720, 0:(1280-UAV_speed_x_4)]
        new_background4[0:720, 0:UAV_speed_x_4] = background4[0:720, (1280-UAV_speed_x_4):1280]
        background4 = new_background4
        UAV_speed_x_4 = -UAV_speed_x_4
    if UAV_speed_y_4 >=0:
        # UAV turn right
        new_background4[0:(720-UAV_speed_y_4),0:1280] = background4[UAV_speed_y_4:720, 0:1280]
        new_background4[(720-UAV_speed_y_4):720, 0:1280] = background4[0:UAV_speed_y_4, 0:1280]
        background4 = new_background4
    else:
        # UAV turn left
        UAV_speed_y_4 = -UAV_speed_y_4
        new_background4[UAV_speed_y_4:720,0:1280] = background4[0:(720-UAV_speed_y_4), 0:1280]
        new_background4[0:UAV_speed_y_4, 0:1280] = background4[(720-UAV_speed_y_4):720, 0:1280]
        background4 = new_background4
        UAV_speed_y_4 = -UAV_speed_y_4


    if UAV_speed_x_5 >= 0:
    # UAV move forward
        new_background5 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background5[0:720, 0:(1280-UAV_speed_x_5)] = background5[0:720, UAV_speed_x_5:1280]
        new_background5[0:720, (1280-UAV_speed_x_5):1280] = background5[0:720, 0:UAV_speed_x_5]
        background5 = new_background5
    else:
        # UAv move backward
        UAV_speed_x_5 = - UAV_speed_x_5
        new_background5 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background5[0:720, UAV_speed_x_5:1280] = background5[0:720, 0:(1280-UAV_speed_x_5)]
        new_background5[0:720, 0:UAV_speed_x_5] = background5[0:720, (1280-UAV_speed_x_5):1280]
        background5 = new_background5
        UAV_speed_x_5 = -UAV_speed_x_5
    if UAV_speed_y_5 >=0:
        # UAV turn right
        new_background5[0:(720-UAV_speed_y_5),0:1280] = background5[UAV_speed_y_5:720, 0:1280]
        new_background5[(720-UAV_speed_y_5):720, 0:1280] = background5[0:UAV_speed_y_5, 0:1280]
        background5 = new_background5
    else:
        # UAV turn left
        UAV_speed_y_5 = -UAV_speed_y_5
        new_background5[UAV_speed_y_5:720,0:1280] = background5[0:(720-UAV_speed_y_5), 0:1280]
        new_background5[0:UAV_speed_y_5, 0:1280] = background5[(720-UAV_speed_y_5):720, 0:1280]
        background5 = new_background5
        UAV_speed_y_5 = -UAV_speed_y_5


    if UAV_speed_x_6 >= 0:
    # UAV move forward
        new_background6 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background6[0:720, 0:(1280-UAV_speed_x_6)] = background6[0:720, UAV_speed_x_6:1280]
        new_background6[0:720, (1280-UAV_speed_x_6):1280] = background6[0:720, 0:UAV_speed_x_6]
        background6 = new_background6
    else:
        # UAv move backward
        UAV_speed_x_6 = - UAV_speed_x_6
        new_background6 = np.zeros((720,1280,3), dtype=np.uint8)
        new_background6[0:720, UAV_speed_x_6:1280] = background6[0:720, 0:(1280-UAV_speed_x_6)]
        new_background6[0:720, 0:UAV_speed_x_6] = background6[0:720, (1280-UAV_speed_x_6):1280]
        background6 = new_background6
        UAV_speed_x_6 = -UAV_speed_x_6
    if UAV_speed_y_6 >=0:
        # UAV turn right
        new_background6[0:(720-UAV_speed_y_6),0:1280] = background6[UAV_speed_y_6:720, 0:1280]
        new_background6[(720-UAV_speed_y_6):720, 0:1280] = background6[0:UAV_speed_y_6, 0:1280]
        background6 = new_background6
    else:
        # UAV turn left
        UAV_speed_y_6 = -UAV_speed_y_6
        new_background6[UAV_speed_y_6:720,0:1280] = background6[0:(720-UAV_speed_y_6), 0:1280]
        new_background6[0:UAV_speed_y_6, 0:1280] = background6[(720-UAV_speed_y_6):720, 0:1280]
        background6 = new_background6
        UAV_speed_y_6 = -UAV_speed_y_6

    cur_scene1 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene1 = copy.copy(background1)

    cur_scene2 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene2 = copy.copy(background2)
    
    cur_scene3 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene3 = copy.copy(background3)

    cur_scene4 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene4 = copy.copy(background4)

    cur_scene5 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene5 = copy.copy(background5)
    
    cur_scene6 = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene6 = copy.copy(background6)    

    ###### Animation UAV Flight ###########  BLOCK_END ############

    #Draw info of UAV
    avoid_vehicle(list_vehicle1)
    # avoid_vehicle(list_vehicle2)
    # avoid_vehicle(list_vehicle3)

    for vehicle in vehicles:
        if (uav[0].locationx_uav-vehicle.current_coordinate[0])  < 700 and (uav[0].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene1 = drawVehicle(cur_scene1, (vehicle.current_coordinate[0]-uav[0].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:    
                cur_scene1 = cv2.putText(cur_scene1, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[0].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()
                vehicle.UAVturnRight(UAV_speed_y_1)
                vehicle.UAVmoveForward(UAV_speed_x_1)   
        
        if (uav[1].locationx_uav-vehicle.current_coordinate[0])  <700 and (uav[1].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene2 = drawVehicle(cur_scene2, (vehicle.current_coordinate[0]-uav[1].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:   
                cur_scene2 = cv2.putText(cur_scene2, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[1].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()         
                vehicle.UAVturnRight(UAV_speed_y_2)
                vehicle.UAVmoveForward(UAV_speed_x_2)

        if (uav[2].locationx_uav-vehicle.current_coordinate[0])  <700 and (uav[2].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene3 = drawVehicle(cur_scene3, (vehicle.current_coordinate[0]-uav[2].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:   
                cur_scene3 = cv2.putText(cur_scene3, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[2].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()         
                vehicle.UAVturnRight(UAV_speed_y_3)
                vehicle.UAVmoveForward(UAV_speed_x_3)

        if (uav[3].locationx_uav-vehicle.current_coordinate[0])  <700 and (uav[3].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene4 = drawVehicle(cur_scene4, (vehicle.current_coordinate[0]-uav[3].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:   
                cur_scene4 = cv2.putText(cur_scene4, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[3].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()         
                vehicle.UAVturnRight(UAV_speed_y_4)
                vehicle.UAVmoveForward(UAV_speed_x_4)

        if (uav[4].locationx_uav-vehicle.current_coordinate[0])  <700 and (uav[4].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene5 = drawVehicle(cur_scene5, (vehicle.current_coordinate[0]-uav[4].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:   
                cur_scene5 = cv2.putText(cur_scene5, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[4].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()         
                vehicle.UAVturnRight(UAV_speed_y_5)
                vehicle.UAVmoveForward(UAV_speed_x_5)

        if (uav[5].locationx_uav-vehicle.current_coordinate[0])  <700 and (uav[5].locationx_uav-vehicle.current_coordinate[0]) > -700:

            cur_scene6 = drawVehicle(cur_scene6, (vehicle.current_coordinate[0]-uav[5].locationx_uav)+640,vehicle.current_coordinate[1], vehicle.vehicletype, vehicle.color)
            if vehicle.weight > 0:   
                cur_scene6 = cv2.putText(cur_scene6, str(float(vehicle.weight)), ((vehicle.current_coordinate[0]-uav[5].locationx_uav)+640,vehicle.current_coordinate[1]), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
                vehicle.update()         
                vehicle.UAVturnRight(UAV_speed_y_6)
                vehicle.UAVmoveForward(UAV_speed_x_6)
        else : 
            vehicle.update()

    for i in range(len(uav)) :
        for reject in range(len(list_reject_main)) :
            print(list_reject_main)
            if (uav[i].locationx_uav-vehicles[list_reject_main[reject]].current_coordinate[0])  <640 and (uav[i].locationx_uav-vehicles[list_reject_main[reject]].current_coordinate[0]) > -640 :
                if (i==1) :
                    print("aaaa")
                    if (list_reject_main[reject]/10==0) :
                        print("uuu")
                        list_vehicle2.append(vehicles[list_reject_main[reject]])
                        list_vehicle_identify2.append(list_reject_main[reject]%10)
                        p_1[list_reject_main[reject]%10] = 1
                        print("               ",utility2)
                        for reject_1 in range(len(list_reject_1)) :
                            if list_reject_1[reject_1] == list_reject_main[reject] :
                                del list_reject_1[reject_1]
                                del list_reject_main[reject]
                                break
                        break
                if (i==2) :
                    print("aaaa")
                    if (list_reject_main[reject]/10==0) :
                        print("uuu")
                        list_vehicle3.append(vehicles[list_reject_main[reject]])
                        list_vehicle_identify3.append(list_reject_main[reject]%10)
                        p_1[list_reject_main[reject]%10] = 1
                        utility3 = +  vehicles[list_reject_main[reject]].weight
                        for reject_2 in range(len(list_reject_2)) :
                            if list_reject_2[reject_2] == list_reject_main[reject] :
                                del list_reject_2[reject_2]
                                del list_reject_main[reject]
                                break
                        break
                if (i==3) :
                    print("aaaa")
                    if (list_reject_main[reject]/10==0) :
                        print("uuu")
                        list_vehicle4.append(vehicles[list_reject_main[reject]])
                        list_vehicle_identify4.append(list_reject_main[reject]%10)
                        p_1[list_reject_main[reject]%10] = 1
                        utility4 = +  vehicles[list_reject_main[reject]].weight
                        for reject_3 in range(len(list_reject_3)) :
                            if list_reject_3[reject_3] == list_reject_main[reject] :
                                del list_reject_3[reject_3]
                                del list_reject_main[reject]
                                break
                        break
                if (i==4) :
                    print("aaaa")
                    if (list_reject_main[reject]/10==0) :
                        print("uuu")
                        list_vehicle5.append(vehicles[list_reject_main[reject]])
                        list_vehicle_identify5.append(list_reject_main[reject]%10)
                        p_1[list_reject_main[reject]%10] = 1
                        utility5 = +  vehicles[list_reject_main[reject]].weight
                        for reject_4 in range(len(list_reject_4)) :
                            if list_reject_4[reject_4] == list_reject_main[reject] :
                                del list_reject_4[reject_4]
                                del list_reject_main[reject]
                                break
                        break

                if (i==5) :
                    print("aaaa")
                    if (list_reject_main[reject]/10==0) :
                        print("uuu")
                        list_vehicle6.append(vehicles[list_reject_main[reject]])
                        list_vehicle_identify6.append(list_reject_main[reject]%10)
                        p_1[list_reject_main[reject]%10] = 1
                        utility6 = +  vehicles[list_reject_main[reject]].weight
                        for reject_5 in range(len(list_reject_5)) :
                            if list_reject_5[reject_5] == list_reject_main[reject] :
                                del list_reject_5[reject_5]
                                del list_reject_main[reject]
                                break
                        break

    print("new : ",list_reject_main,list_reject_1)
            # cv2.imshow("cur_scene2",cur_scene2)
             
    ROI_1,num_max_1,num_min_1 = findROI(list_vehicle_coordinates1,list_vehicle1,uav[0])
    ROI_2,num_max_2,num_min_2 = findROI(list_vehicle_coordinates2,list_vehicle2,uav[1])
    ROI_3,num_max_3,num_min_3 = findROI(list_vehicle_coordinates3,list_vehicle3,uav[2])
    ROI_4,num_max_4,num_min_4 = findROI(list_vehicle_coordinates4,list_vehicle4,uav[3])
    ROI_5,num_max_5,num_min_5 = findROI(list_vehicle_coordinates5,list_vehicle5,uav[4])
    ROI_6,num_max_6,num_min_6 = findROI(list_vehicle_coordinates6,list_vehicle6,uav[5])

    xcenter_ROI_1 = int((ROI_1[0][0]+ROI_1[1][0])/2)
    ycenter_ROI_1 = int((ROI_1[0][1]+ROI_1[1][1])/2)
    center_ROI_1 =(xcenter_ROI_1, ycenter_ROI_1)

    xcenter_ROI_2 = int((ROI_2[0][0]+ROI_2[1][0])/2)
    ycenter_ROI_2 = int((ROI_2[0][1]+ROI_2[1][1])/2)
    center_ROI_2 =(xcenter_ROI_2, ycenter_ROI_2)

    xcenter_ROI_3 = int((ROI_3[0][0]+ROI_3[1][0])/2)
    ycenter_ROI_3 = int((ROI_3[0][1]+ROI_3[1][1])/2)
    center_ROI_3 =(xcenter_ROI_3, ycenter_ROI_3)

    xcenter_ROI_4 = int((ROI_4[0][0]+ROI_4[1][0])/2)
    ycenter_ROI_4 = int((ROI_4[0][1]+ROI_4[1][1])/2)
    center_ROI_4 =(xcenter_ROI_4, ycenter_ROI_4)

    xcenter_ROI_5 = int((ROI_5[0][0]+ROI_5[1][0])/2)
    ycenter_ROI_5 = int((ROI_5[0][1]+ROI_5[1][1])/2)
    center_ROI_5 =(xcenter_ROI_5, ycenter_ROI_5)

    xcenter_ROI_6 = int((ROI_6[0][0]+ROI_6[1][0])/2)
    ycenter_ROI_6 = int((ROI_6[0][1]+ROI_6[1][1])/2)
    center_ROI_6 =(xcenter_ROI_6, ycenter_ROI_6)
    # print(list_vehicle_coordinates2)
    print(uav[1].locationx_uav,uav[1].locationy_uav)
    
    # drawROI(cur_scene1, ROI_1[0], ROI_1[1],(255,0,0))
    # drawROI(cur_scene2, ROI_2[0], ROI_2[1],(255,0,0))

    # print("cur :\n",list_vehicle_coordinates)
    # cur_scene = cv2.putText(cur_scene, 'CurrentROI', ROI[0], cv2.FONT_HERSHEY_SIMPLEX , 0.6, (0,255,255), 2, cv2.LINE_AA)
    # print("current",ROI[0],ROI[1])
    

    list_vehicle_predict_1= updateVehicle_predict(controll_1,list_vehicle1,uav[0])
    list_vehicle_predict_2= updateVehicle_predict(controll_2,list_vehicle2,uav[1])
    list_vehicle_predict_3= updateVehicle_predict(controll_3,list_vehicle3,uav[2])
    list_vehicle_predict_4= updateVehicle_predict(controll_4,list_vehicle4,uav[3])
    list_vehicle_predict_5= updateVehicle_predict(controll_5,list_vehicle5,uav[4])
    list_vehicle_predict_6= updateVehicle_predict(controll_6,list_vehicle6,uav[5])

    ROI_predict_1 , num_max_1 , num_min_1= findROI(list_vehicle_predict_1,list_vehicle1,uav[0])
    ROI_predict_2 , num_max_2 , num_min_2= findROI(list_vehicle_predict_2,list_vehicle2,uav[1])
    ROI_predict_3 , num_max_3 , num_min_3= findROI(list_vehicle_predict_3,list_vehicle3,uav[2])
    ROI_predict_4 , num_max_4 , num_min_4= findROI(list_vehicle_predict_4,list_vehicle4,uav[3])
    ROI_predict_5 , num_max_5 , num_min_5= findROI(list_vehicle_predict_5,list_vehicle5,uav[4])
    ROI_predict_6 , num_max_6 , num_min_6= findROI(list_vehicle_predict_6,list_vehicle6,uav[5])

    # print("num_1 : ",num_max_1,num_min_1)
    # print("num_2 : ",num_max_2,num_min_2)
    # print("num reject ",num_reject)
    xcenter_ROI_predict_1 = int((ROI_predict_1[0][0]+ROI_predict_1[1][0])/2)
    ycenter_ROI_predict_1 = int((ROI_predict_1[0][1]+ROI_predict_1[1][1])/2)
    center_ROI_predict_1 =(xcenter_ROI_predict_1, ycenter_ROI_predict_1)

    xcenter_ROI_predict_2 = int((ROI_predict_2[0][0]+ROI_predict_2[1][0])/2)
    ycenter_ROI_predict_2 = int((ROI_predict_2[0][1]+ROI_predict_2[1][1])/2)
    center_ROI_predict_2 =(xcenter_ROI_predict_2, ycenter_ROI_predict_2)  

    xcenter_ROI_predict_3 = int((ROI_predict_3[0][0]+ROI_predict_3[1][0])/2)
    ycenter_ROI_predict_3 = int((ROI_predict_3[0][1]+ROI_predict_3[1][1])/2)
    center_ROI_predict_3 =(xcenter_ROI_predict_3, ycenter_ROI_predict_3)   

    xcenter_ROI_predict_4 = int((ROI_predict_4[0][0]+ROI_predict_4[1][0])/2)
    ycenter_ROI_predict_4 = int((ROI_predict_4[0][1]+ROI_predict_4[1][1])/2)
    center_ROI_predict_4 =(xcenter_ROI_predict_4, ycenter_ROI_predict_4)  
 
    xcenter_ROI_predict_5 = int((ROI_predict_5[0][0]+ROI_predict_5[1][0])/2)
    ycenter_ROI_predict_5 = int((ROI_predict_5[0][1]+ROI_predict_5[1][1])/2)
    center_ROI_predict_5 =(xcenter_ROI_predict_5, ycenter_ROI_predict_5)     

    xcenter_ROI_predict_6 = int((ROI_predict_6[0][0]+ROI_predict_6[1][0])/2)
    ycenter_ROI_predict_6 = int((ROI_predict_6[0][1]+ROI_predict_6[1][1])/2)
    center_ROI_predict_6 =(xcenter_ROI_predict_6, ycenter_ROI_predict_6) 
    # print("predicted",ROI_predict[0],ROI_predict[1])
    # print("predicted:\n",list_vehicle_predict)

    drawROI(cur_scene1, ROI_predict_1[0], ROI_predict_1[1],(255,255,0))
    drawROI(cur_scene2, ROI_predict_2[0], ROI_predict_2[1],(255,255,0))
    drawROI(cur_scene3, ROI_predict_3[0], ROI_predict_3[1],(255,255,0))
    drawROI(cur_scene4, ROI_predict_4[0], ROI_predict_4[1],(255,255,0))
    drawROI(cur_scene5, ROI_predict_5[0], ROI_predict_5[1],(255,255,0))
    drawROI(cur_scene6, ROI_predict_6[0], ROI_predict_6[1],(255,255,0))

    # print(num_reject_1)
    print("vehicle 1 : ",vehicles[0].current_coordinate[0])
    # print("ROI 2 : ",ROI_predict_2[0],ROI_predict_2[1])
    if ((ROI_predict_1[1][0] - ROI_predict_1[0][0]) > 1200) :
        num_reject_1=time_cumulative_utility(num_max_1,num_min_1,list_must_vehicle1,p_1,0)
        controll_1 = 1
        # print("num reject 1 :",num_reject_1)
    if ((ROI_predict_2[1][0] - ROI_predict_2[0][0]) > 1200) :
        num_reject_2=time_cumulative_utility(num_max_2,num_min_2,list_must_vehicle2,p_2,5)
        controll_2 = 1   
        # print("num reject 2 :",num_reject_2)
    if ((ROI_predict_3[1][0] - ROI_predict_3[0][0]) > 1200) :
        num_reject_3=time_cumulative_utility(num_max_3,num_min_3,list_must_vehicle3,p_3,10)
        controll_3 = 1   

    if ((ROI_predict_4[1][0] - ROI_predict_4[0][0]) > 1200) :
        num_reject_4=time_cumulative_utility(num_max_4,num_min_4,list_must_vehicle4,p_4,15)
        controll_4 = 1  
    
    if ((ROI_predict_5[1][0] - ROI_predict_5[0][0]) > 1200) :
        num_reject_5=time_cumulative_utility(num_max_5,num_min_5,list_must_vehicle5,p_5,20)
        controll_5 = 1  
        print(num_reject_5)
    if ((ROI_predict_6[1][0] - ROI_predict_6[0][0]) > 1200) :
        num_reject_6=time_cumulative_utility(num_max_6,num_min_6,list_must_vehicle6,p_6,25)
        controll_6 = 1      


        # print("num reject 3 :",num_reject_3)
    # print(controll)
    cur_scene1 = drawUAVInfo(cur_scene1, (UAV_speed_x_1, UAV_speed_y_1), (uav[0].locationx_uav,uav[0].locationy_uav), 70, count,len(list_vehicle1),len(list_reject_1),utility1,uav[0].identify)
    cur_scene2 = drawUAVInfo(cur_scene2, (UAV_speed_x_2, UAV_speed_y_2), (uav[1].locationx_uav,uav[1].locationy_uav), 70, count,len(list_vehicle2),len(list_reject_2),utility2,uav[1].identify)
    cur_scene3 = drawUAVInfo(cur_scene3, (UAV_speed_x_3, UAV_speed_y_3), (uav[2].locationx_uav,uav[2].locationy_uav), 70, count,len(list_vehicle3),len(list_reject_3),utility3,uav[2].identify)
    cur_scene4 = drawUAVInfo(cur_scene4, (UAV_speed_x_4, UAV_speed_y_4), (uav[3].locationx_uav,uav[3].locationy_uav), 70, count,len(list_vehicle4),len(list_reject_4),utility4,uav[3].identify)
    cur_scene5 = drawUAVInfo(cur_scene5, (UAV_speed_x_5, UAV_speed_y_5), (uav[4].locationx_uav,uav[4].locationy_uav), 70, count,len(list_vehicle5),len(list_reject_5),utility5,uav[4].identify)
    cur_scene6 = drawUAVInfo(cur_scene6, (UAV_speed_x_6, UAV_speed_y_6), (uav[5].locationx_uav,uav[5].locationy_uav), 70, count,len(list_vehicle6),len(list_reject_6),utility6,uav[5].identify)

    # cur_scene3 = drawUAVInfo(cur_scene3, (UAV_speed_x, UAV_speed_y), (locationx,locationy), 70, count,len(list_vehicle),5-len(list_vehicle),utility,uav[2].identify)
    # cur_scene4 = drawUAVInfo(cur_scene4, (UAV_speed_x, UAV_speed_y), (locationx,locationy), 70, count,len(list_vehicle),5-len(list_vehicle),utility,uav[3].identify)
    # cur_scene5 = drawUAVInfo(cur_scene5, (UAV_speed_x, UAV_speed_y), (locationx,locationy), 70, count,len(list_vehicle),5-len(list_vehicle),utility,uav[4].identify)
    # cur_scene6 = drawUAVInfo(cur_scene6, (UAV_speed_x, UAV_speed_y), (locationx,locationy), 70, count,len(list_vehicle),5-len(list_vehicle),utility,uav[5].identify)

    # cur_scene = cv2.putText(cur_scene, 'PredictiveROI', ROI_predict[1], cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,0), 2, cv2.LINE_AA)
    # if (count == 200) :
    #     vehicles[0].speedStart = 0
    #     vehicles[0].accel = 0
    # if (count == 300) :
    #     vehicles[0].speedStart = 17
    #     vehicles[0].accel = 0.0013
    
    if count> 10 :
        UAV_speed_x_1 += int((xcenter_ROI_predict_1 - 800)*0.01)
        UAV_speed_y_1 += int((ycenter_ROI_predict_1 - 360)*0.01)
        UAV_speed_x_2 += int((xcenter_ROI_predict_2 - 800)*0.01)
        UAV_speed_y_2 += int((ycenter_ROI_predict_2 - 360)*0.01)
        UAV_speed_x_3 += int((xcenter_ROI_predict_3 - 640)*0.01)
        UAV_speed_y_3 += int((ycenter_ROI_predict_3 - 360)*0.01)
        UAV_speed_x_4 += int((xcenter_ROI_predict_4 - 900)*0.01)
        UAV_speed_y_4 += int((ycenter_ROI_predict_4 - 360)*0.01)
        UAV_speed_x_5 += int((xcenter_ROI_predict_5 - 640)*0.01)
        UAV_speed_y_5 += int((ycenter_ROI_predict_5 - 360)*0.01)
        UAV_speed_x_6 += int((xcenter_ROI_predict_6 - 640)*0.01)
        UAV_speed_y_6 += int((ycenter_ROI_predict_6 - 360)*0.01)
        # if xcenter_ROI_predict_1 - 640 > -5 and UAV_speed_x_1 <= uav[0].speedMax:
        # UAV_speed_x_1+=int(0.02 * (xcenter_ROI_predict_1 - 640)) + int(0.01 * ((xcenter_ROI_predict_1 - 640) - pid_last_UAV_1_d_x_error))
        # pid_last_UAV_1_d_x_error = (xcenter_ROI_predict_1 - 640)
        # if xcenter_ROI_predict_1 - 640 < 5 and UAV_speed_x_1 >=0:
        #     UAV_speed_x_1-=1
        # if ycenter_ROI_predict_1 - 360 > -5 and UAV_speed_y_1 <= 1:
        # UAV_speed_y_1+=int(0.01 * (ycenter_ROI_predict_1 - 360)) + int(0.01 * ((ycenter_ROI_predict_1 - 360) - pid_last_UAV_1_d_y_error))
        # pid_last_UAV_1_d_y_error = (ycenter_ROI_predict_1 - 360)
        # if ycenter_ROI_predict_1 - 360 < 5 and UAV_speed_y_1 >=-1:
        #     UAV_speed_y_1-=1
        # UAV_speed_x_2+=int(0.02 * (xcenter_ROI_predict_2 - 640)) + int(0.01 * ((xcenter_ROI_predict_2 - 640) - pid_last_UAV_2_d_x_error))
        # pid_last_UAV_2_d_x_error = (xcenter_ROI_predict_2 - 640)

        # UAV_speed_y_2+=int(0.01 * (ycenter_ROI_predict_2 - 360)) + int(0.01 * ((ycenter_ROI_predict_2 - 360) - pid_last_UAV_2_d_y_error))
        # pid_last_UAV_2_d_y_error = (ycenter_ROI_predict_2 - 360)

        # UAV_speed_x_3+=int(0.03 * (xcenter_ROI_predict_3 - 640)) + int(0.01 * ((xcenter_ROI_predict_3 - 640) - pid_last_UAV_3_d_x_error))
        # pid_last_UAV_3_d_x_error = (xcenter_ROI_predict_3 - 640)

        # UAV_speed_y_3+=int(0.01 * (ycenter_ROI_predict_3 - 360)) + int(0.01 * ((ycenter_ROI_predict_3 - 360) - pid_last_UAV_3_d_y_error))
        # pid_last_UAV_3_d_y_error = (ycenter_ROI_predict_3 - 360)

        # UAV_speed_x_4+=int(0.03 * (xcenter_ROI_predict_4 - 640)) + int(0.01 * ((xcenter_ROI_predict_4 - 640) - pid_last_UAV_4_d_x_error))
        # pid_last_UAV_4_d_x_error = (xcenter_ROI_predict_4 - 640)

        # UAV_speed_y_4+=int(0.01 * (ycenter_ROI_predict_4 - 360)) + int(0.01 * ((ycenter_ROI_predict_4 - 360) - pid_last_UAV_4_d_y_error))
        # pid_last_UAV_4_d_y_error = (ycenter_ROI_predict_4 - 360)

        # UAV_speed_x_5+=int(0.03 * (xcenter_ROI_predict_5 - 640)) + int(0.01 * ((xcenter_ROI_predict_5 - 640) - pid_last_UAV_5_d_x_error))
        # pid_last_UAV_5_d_x_error = (xcenter_ROI_predict_5 - 640)

        # UAV_speed_y_5+=int(0.04 * (ycenter_ROI_predict_5 - 360)) + int(0.01 * ((ycenter_ROI_predict_5 - 360) - pid_last_UAV_5_d_y_error))
        # pid_last_UAV_5_d_y_error = (ycenter_ROI_predict_5 - 360)

        # UAV_speed_x_6+=int(0.02 * (xcenter_ROI_predict_6 - 640)) + int(0.01 * ((xcenter_ROI_predict_6 - 640) - pid_last_UAV_6_d_x_error))
        # pid_last_UAV_6_d_x_error = (xcenter_ROI_predict_6 - 640)

        # UAV_speed_y_6+=int(0.01 * (ycenter_ROI_predict_6 - 360)) + int(0.01 * ((ycenter_ROI_predict_6 - 360) - pid_last_UAV_6_d_y_error))
        # pid_last_UAV_6_d_y_error = (ycenter_ROI_predict_6 - 360)

        # if xcenter_ROI_predict_2 - 640 > -5 and UAV_speed_x_2 <= uav[1].speedMax:
        #     UAV_speed_x_2+=2
        # if xcenter_ROI_predict_2 - 640 < 5 and UAV_speed_x_2 >=0:
        #     UAV_speed_x_2-=1
        # if ycenter_ROI_predict_2 - 360 > -5 and UAV_speed_y_2 <= 1:
        #     UAV_speed_y_2+=1
        # if ycenter_ROI_predict_2 - 360 < 5 and UAV_speed_y_2 >=-1:
        #     UAV_speed_y_2-=1

        # if xcenter_ROI_predict_3 - 640 > -5 and UAV_speed_x_3 <= uav[2].speedMax:
        #     UAV_speed_x_3+=2
        # if xcenter_ROI_predict_3 - 640 < 5 and UAV_speed_x_3 >=0:
        #     UAV_speed_x_3-=1
        # if ycenter_ROI_predict_3 - 360 > -5 and UAV_speed_y_3 <= 1:
        #     UAV_speed_y_3+=1
        # if ycenter_ROI_predict_3 - 360 < 5 and UAV_speed_y_3 >=-1:
        #     UAV_speed_y_3-=1

        # if xcenter_ROI_predict_4 - 640 > -5 and UAV_speed_x_4 <= uav[3].speedMax:
        #     UAV_speed_x_4+=2
        # if xcenter_ROI_predict_4 - 640 < 5 and UAV_speed_x_4 >=0:
        #     UAV_speed_x_4-=1
        # if ycenter_ROI_predict_4 - 360 > -5 and UAV_speed_y_4 <= 1:
        #     UAV_speed_y_4+=1
        # if ycenter_ROI_predict_4 - 360 < 5 and UAV_speed_y_4 >=-1:
        #     UAV_speed_y_4-=1

        # if xcenter_ROI_predict_5 - 640 > -5 and UAV_speed_x_5 <= uav[4].speedMax:
        #     UAV_speed_x_5+=2
        # if xcenter_ROI_predict_5 - 640 < 5 and UAV_speed_x_5 >=0:
        #     UAV_speed_x_5-=1
        # if ycenter_ROI_predict_5 - 360 > -5 and UAV_speed_y_5 <= 1:
        #     UAV_speed_y_5+=1
        # if ycenter_ROI_predict_5 - 360 < 5 and UAV_speed_y_5 >=-1:
        #     UAV_speed_y_5-=1

        # if xcenter_ROI_predict_6 - 640 > -5 and UAV_speed_x_6 <= uav[5].speedMax:
        #     UAV_speed_x_6+=2
        # if xcenter_ROI_predict_6 - 640 < 5 and UAV_speed_x_6 >=0:
        #     UAV_speed_x_6-=1
        # if ycenter_ROI_predict_6 - 360 > -5 and UAV_speed_y_6 <= 1:
        #     UAV_speed_y_6+=1
        # if ycenter_ROI_predict_6 - 360 < 5 and UAV_speed_y_6 >=-1:
        #     UAV_speed_y_6-=1

        for vehicle in vehicles:
            vehicle.speed_x = 10*vehicle.accel + vehicle.speedStart
            vehicle.speedStart = vehicle.speed_x


    cur_scene_main = concat_tile(   [   [cur_scene1,partition,cur_scene2,partition,cur_scene3],
                                       [cur_scene4,partition,cur_scene5,partition,cur_scene6]  ] )
    dsize_scene = (1280,720) 
    cur_scene_main = cv2.resize(cur_scene_main,dsize_scene)
    cv2.imshow('frame',cur_scene_main)
    output_movie.write(cur_scene_main)
    uav[0].locationx_uav = uav[0].locationx_uav + UAV_speed_x_1
    uav[0].locationy_uav = uav[0].locationy_uav + UAV_speed_y_1

    uav[1].locationx_uav = uav[1].locationx_uav + UAV_speed_x_2
    uav[1].locationy_uav = uav[1].locationy_uav + UAV_speed_y_2

    uav[2].locationx_uav = uav[2].locationx_uav + UAV_speed_x_3
    uav[2].locationy_uav = uav[2].locationy_uav + UAV_speed_y_3

    uav[3].locationx_uav = uav[3].locationx_uav + UAV_speed_x_4
    uav[3].locationy_uav = uav[3].locationy_uav + UAV_speed_y_4

    uav[4].locationx_uav = uav[4].locationx_uav + UAV_speed_x_5
    uav[4].locationy_uav = uav[4].locationy_uav + UAV_speed_y_5

    uav[5].locationx_uav = uav[5].locationx_uav + UAV_speed_x_6
    uav[5].locationy_uav = uav[5].locationy_uav + UAV_speed_y_6

    # print(locationx,locationy)
    time.sleep(0.1)
    end = time.time()
    print("time : ",end-start)
    # # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
del vehicles,list_vehicle_coordinates,list_vehicle,list_vehicle_predict,list_must_vehicle,list_vehicle_predict,locationx,locationy,cur_scene,count,numObj,num_max,num_min

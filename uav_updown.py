import numpy as np 
import cv2
import time
import copy

# Define a vehicle with line trajectory
class lineTrajectory:
    'linear equations: y=ax+b '
    def __init__(self, startx, a, b, vehicletype:'car', color:'red', speed: int):
        self.startx=startx
        self.a=a
        self.b=b
        self.vehicletype=vehicletype
        self.color=color
        self.speed=speed
        # Toa do vehicle trong khung hinh
        self.current_coordinate = (int(startx),int(a*startx+b))
        
        # Location he quy chieu mat dat
        self.location = [self.current_coordinate[0]-1280, self.current_coordinate[1]-720]
        #
        self.trajectory = [self.location]
    
    def update(self):
        # Update position vehicle in frame
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x + self.speed
        new_y = cur_y + self.a*self.speed
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate=new_coordinate

        # Update location and add to trajectory
        cur_lx, cur_ly = self.location
        new_lx = cur_lx + self.speed
        new_ly = self.a*new_lx +self.b
        new_location = [int(new_lx),int(new_ly)]
        self.location = new_location
        # print(self.location)

        # Update trajectory
        self.trajectory.append(self.location)
        # print(self.trajectory)
        

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

    # Predict location using Linear regression
    def predictLocation(self, num_sample = 20):
        # Linear regression
        # Lay mau mac dinh 15 diem du lieu gan nhat
        if len(self.trajectory) > num_sample:
            dataset = self.trajectory[- num_sample:]
            dataset = np.array(dataset)
            # print('DATASET',dataset)
            x_train = dataset[:,0] #outcome
            y_train = dataset[:,1] #outcome
            t_train = np.arange(0,20) #input
            # Tien xu ly du lieu
            x_train = np.array([x_train]).T
            y_train = np.array([y_train]).T
            t_train = np.array([t_train]).T

            one = np.ones((t_train.shape[0], 1))
            t_train_bar = np.concatenate((one, t_train), axis = 1)

            # Calculating weights of the fitting line 
            A = np.dot(t_train_bar.T, t_train_bar)
            bx = np.dot(t_train_bar.T, x_train)
            by = np.dot(t_train_bar.T, y_train)
            w_x = np.dot(np.linalg.pinv(A), bx)
            w_y = np.dot(np.linalg.pinv(A), by)

            # weight x
            w0_x = w_x[0][0]
            w1_x = w_x[1][0]
            # weight y
            w0_y = w_y[0][0]
            w1_y = w_y[1][0]
            
            # Predict location t = 22
            x_predict = int(w1_x*23 + w0_x)
            y_predict = int(w1_y*23 + w0_y)
         
            return [x_predict, y_predict]
        
    def predictCoordinate(self):
        # Location hien tai cua vehicle
        x_location, y_location = self.location
        # Location du doan cua vehicle 
        x_predict_location , y_predict_location = self.predictLocation()
        # Toa do hien tai trong khung hinh
        x_cur_coordinate, y_cur_coordinate = self.current_coordinate
        # Toa do du doan = toa do hien tai + delta(location) 
        x_predict_coordinate = x_cur_coordinate + x_predict_location - x_location
        y_predict_coordinate = y_cur_coordinate + y_predict_location - y_location
        predict_coordinate = (x_predict_coordinate, y_predict_coordinate)
        return predict_coordinate

class uav_class:
    def __init__(self):
        #Khoi tao toa do UAV
        self.location = [0, 0]
        #Van toc
        self.speed_x = 0
        self.speed_y = 0

        #Speed per frame pixel/frame
        self.speed_xint = 0
        self.speed_yint = 0
        #Gia toc
        self.ax = 0
        self.ay = 0
        #Do cao
        self.h = 60
    
    def calculate_acceleration(self):
        return 0

    #Update speed per frame
    def update_speed(self, t = 1):
        # V = Vo + at 
        new_speed = self.speed + self.a*t
        self.speed = new_speed
        self.speed_int = round(self.speed)

#Draw vehicle on image
def drawVehicle(image: np.zeros((1440,2560,3), np.uint8), center_coordinates, vehicletype, color):
    vehicle_icon = cv2.imread('images/' + vehicletype + '-' + color + '.jpg',1)
    height_vehicle, width_vehicle = vehicle_icon.shape[0:2]
    delta_x = int((width_vehicle - 1)/2)
    delta_y = int((height_vehicle -1)/2)
    # print(delta_x, delta_y)
    # print('SHAPE: ',vehicle_icon.shape)
    # Xe nam tron trong khung hinh
    if center_coordinates[0] >= delta_x and center_coordinates[1] >= delta_y and 2560 - center_coordinates[0] > delta_x and 1440 - center_coordinates[1] > delta_y:
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, \
            center_coordinates[0] - delta_x: center_coordinates[0] + delta_x +1] = vehicle_icon
    
    # Xe tran trai khung hinh
    if center_coordinates[0] < delta_x and center_coordinates[0] + delta_x > 0 :
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, 0:center_coordinates[0] + delta_x + 1] = vehicle_icon[:,width_vehicle - center_coordinates[0]-delta_x - 1:width_vehicle]
    # Xe tran phai khung hinh
    if center_coordinates[0] > 2560 - delta_x and center_coordinates[0]-delta_x < 2560:
        image[center_coordinates[1] - delta_y: center_coordinates[1] + delta_y +1, center_coordinates[0] - delta_x:2560] = vehicle_icon[:,0:2560 - center_coordinates[0] + delta_x]
    # Xe tran phai khung hinh
    # if center_coordinates
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


def drawPointVehicle(image: np.zeros((1440,2560,3), np.uint8), _center_coordinates, color):

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
    for vehicle_coordinate in list_vehicle_coordinates:
        if x_max < vehicle_coordinate[0]:
            x_max = vehicle_coordinate[0]
        if x_min > vehicle_coordinate[0]:
            x_min = vehicle_coordinate[0]

        if y_max < vehicle_coordinate[1]:
            y_max = vehicle_coordinate[1]
        if y_min> vehicle_coordinate[1]:
            y_min = vehicle_coordinate[1]

        start_point = (x_min, y_min)
        end_point = (x_max, y_max)
    return (start_point, end_point)

def drawUAVInfo(image, speed, location, altitude, _time, num_tracking=3, num_missing=0):
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

background = cv2.imread('images/ground4.jpg',1)
# crop_background = background[360:1080,640:1920]

#Khoi tao UAV
uav = uav_class()
uav.speed_x = 5
uav.speed_y = 0

# Initialization vehicles
vehicles =[]
vehicles.append( lineTrajectory(700, 0.15, 690, 'car', 'yellow', 12) )
vehicles.append( lineTrajectory(300, 0, 620, 'car', 'red', 16) )
vehicles.append( lineTrajectory(1000, 0, 520, 'truck', 'green', 14) )
vehicles.append( lineTrajectory(-50, 0, 700, 'car1', 'white', 16) )
vehicles.append( lineTrajectory(-500, 0, 800, 'truck', 'cyan', 16) )

UAV_speed_x = 5
UAV_speed_y = 1
h = 60
delta_h = 1
count=0
#Running
while(1):
    ###### Animation UAV Flight ###########  BLOCK_START ############
    if UAV_speed_x >= 0:
    # UAV move forward
        new_background = np.zeros((1440,2560,3), dtype=np.uint8)
        new_background[0:1440, 0:(2560-UAV_speed_x)] = background[0:1440, UAV_speed_x:2560]
        new_background[0:1440, (2560-UAV_speed_x):2560] = background[0:1440, 0:UAV_speed_x]
        background = new_background
    else:
        # UAv move backward
        UAV_speed_x = - UAV_speed_x
        new_background = np.zeros((1440,2560,3), dtype=np.uint8)
        new_background[0:1440, UAV_speed_x:2560] = background[0:1440, 0:(2560-UAV_speed_x)]
        new_background[0:1440, 0:UAV_speed_x] = background[0:1440, (2560-UAV_speed_x):2560]
        background = new_background
        UAV_speed_x = -UAV_speed_x
    if UAV_speed_y >=0:
        # UAV turn right
        new_background[0:(1440-UAV_speed_y),0:2560] = background[UAV_speed_y:1440, 0:2560]
        new_background[(1440-UAV_speed_y):1440, 0:2560] = background[0:UAV_speed_y, 0:2560]
        cur_scene = copy.copy(background)
    else:
        # UAV turn left
        UAV_speed_y = -UAV_speed_y
        new_background[UAV_speed_y:1440,0:2560] = background[0:(1440-UAV_speed_y), 0:2560]
        new_background[0:UAV_speed_y, 0:2560] = background[(1440-UAV_speed_y):1440, 0:2560]
        UAV_speed_y = -UAV_speed_y
    cur_scene = np.zeros((1440,2560,3), dtype=np.uint8)
    cur_scene = copy.copy(background)

    #Draw all vehicles
    for vehicle in vehicles:

        cur_scene = drawVehicle(cur_scene, vehicle.current_coordinate, vehicle.vehicletype, vehicle.color)
        # cur_scene = drawPointVehicle(cur_scene, vehicle.current_coordinate, (0,0,255))
        vehicle.update()
        vehicle.UAVturnRight(UAV_speed_y)
        vehicle.UAVmoveForward(UAV_speed_x)

    x_crop = int((h/60*1280)/2)
    y_crop = int((h/60*720)/2)
    crop_background = cur_scene[720-y_crop:720+y_crop,1280-x_crop:1280+x_crop]
    view_screen = cv2.resize(crop_background,(1280, 720))
    ###### Animation UAV Flight ###########  BLOCK_END ############

    ##### UAV change Altitude ############3
    if count < 100:
        h = h+delta_h
    if h > 120:
        h=120
    if count > 100:
        h=h-delta_h
    if h <60:
        h=60

    count+=1
    cv2.imshow('frame',view_screen)
    time.sleep(0.1)
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()

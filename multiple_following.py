# Create a simulation tool about multiple-vehicle tracking using UAV
# Author: HaiLV
# Email: levanhai2206@gmail.com
# Date: 10/06/2020

import cv2
import numpy as np
import time

# Define a vehicle with line trajectory
class lineTrajectory:
    'linear equations: y=ax+b '
    def __init__(self, startx, a, b, color: (0, 0, 0), speed: int):
        self.startx=startx
        self.a=a
        self.b=b
        self.color=color
        self.speed=speed
        self.current_coordinate = (int(startx),int(a*startx+b))
        self.trajectory=[self.current_coordinate]
    
    def update(self):
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x + self.speed
        new_y = self.a*new_x +self.b
        new_coordinate = (int(new_x),int(new_y))
        self.trajectory.append(new_coordinate)
        self.current_coordinate=new_coordinate
    
    def UAVmoveForward(self, UAVspeed):
        cur_x, cur_y = self.current_coordinate
        new_x = cur_x - UAVspeed
        new_y = cur_y
        new_coordinate = (int(new_x),int(new_y))
        self.current_coordinate = new_coordinate
    
    # def UAVturnRight(self, UAVspeed)

def drawVehicle(image: np.zeros((640,960,3), np.uint8), center_coordinates, color):

    # Radius of circle 
    radius = 20
    
    # Line thickness of -1 px 
    thickness = -1
    
    # Using cv2.circle() method 
    # Draw a circle of red color of thickness -1 px 
    image = cv2.circle(image, center_coordinates, radius, color, thickness) 
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

def drawROI(image, start, end):
    # Blue color in BGR 
    color = (0, 255, 255) 
    
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

def drawPointVehicle(image: np.zeros((640,960,3), np.uint8), center_coordinates, color):

    # Radius of circle 
    radius = 20
    
    # Line thickness of -1 px 
    thickness = -1
    
    # Using cv2.circle() method 
    # Draw a circle of red color of thickness -1 px 
    image = cv2.circle(image, center_coordinates, radius, color, thickness) 
    return image

#Output video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_movie = cv2.VideoWriter('simulation.mp4', fourcc, 10, (960, 640))

#initialization vehicles   
vehicle1 = lineTrajectory(0, 0.1, 150, (255, 0, 0), 10)
vehicle2 = lineTrajectory(50, 0.115, 300, (0, 255, 0), 13)
vehicle3 = lineTrajectory(20, 0.11, 420, (0, 0, 255), 11)
vehicle4 = lineTrajectory(-40, 0.099, 320, (180, 80, 255), 10)

i=0
uav_speed = 0
uav_icon=cv2.imread('image.jpg',1)

#ACTION 
while(1):
    i+=1
    frame = np.zeros((640,960,3), dtype=np.uint8)
    
    #Set Speed of Uav
    if i==50: 
        uav_speed = 6
    #Put header text
    #Add icon UAV
    frame[0:40,400:466] = uav_icon
    
    frame = cv2.putText(frame, 'Speed: '+str(uav_speed)+'m/s', (500,30), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,255), 1, cv2.LINE_AA)
    frame = cv2.putText(frame, 'H: 70m', (680,30), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,255), 1, cv2.LINE_AA)
    frame = cv2.putText(frame, 'Time: '+str(i), (780,30), cv2.FONT_HERSHEY_SIMPLEX , 0.6, (255,255,255), 1, cv2.LINE_AA)

    list_vehicle = [vehicle1.current_coordinate, vehicle2.current_coordinate, vehicle3.current_coordinate, vehicle4.current_coordinate]
    
    frame = drawVehicle(frame, vehicle1.current_coordinate, vehicle1.color)
    vehicle1.update()
    vehicle1.UAVmoveForward(uav_speed)

    frame = drawVehicle(frame, vehicle2.current_coordinate, vehicle2.color)
    vehicle2.update()
    vehicle2.UAVmoveForward(uav_speed)

    frame = drawVehicle(frame, vehicle3.current_coordinate, vehicle3.color)
    vehicle3.update()
    vehicle3.UAVmoveForward(uav_speed)

    frame = drawVehicle(frame, vehicle4.current_coordinate, vehicle4.color)
    vehicle4.update()
    vehicle4.UAVmoveForward(uav_speed)

    ROI = findROI(list_vehicle)
    xcenter_ROI = int((ROI[0][0]+ROI[1][0])/2)
    ycenter_ROI = int((ROI[0][1]+ROI[1][1])/2)
    center_ROI =(xcenter_ROI, ycenter_ROI)
    if i >20:
        drawROI(frame, ROI[0], ROI[1])
        frame = cv2.circle(frame, (480,320), 7, (255,255,255), 2) 
    if i >30:
        drawMotionVector(frame, (480,320), center_ROI)
    
    if i>50:
        if xcenter_ROI - 480 > -20 and uav_speed <= 15:
            uav_speed+=2
        if xcenter_ROI - 480 < 20 and uav_speed >=8:
            uav_speed-=1
    
    
    print(uav_speed)
    print(findROI(list_vehicle))
    print(vehicle1.current_coordinate)

    cv2.imshow('frame',frame)
    output_movie.write(frame)
    time.sleep(0.2)
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
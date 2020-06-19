import numpy as np 
import cv2
import time
import copy

background0 = cv2.imread('images/ground4.jpg',1)
background = background0[360:1080,640:1920]

UAV_speed_x = 5
UAV_speed_y = 1
#Running
while(1):
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
        cur_scene = copy.copy(background)
    else:
        # UAV turn left
        UAV_speed_y = -UAV_speed_y
        new_background[UAV_speed_y:720,0:1280] = background[0:(720-UAV_speed_y), 0:1280]
        new_background[0:UAV_speed_y, 0:1280] = background[(720-UAV_speed_y):720, 0:1280]
        UAV_speed_y = -UAV_speed_y
    cur_scene = np.zeros((720,1280,3), dtype=np.uint8)
    cur_scene = copy.copy(background)
    ###### Animation UAV Flight ###########  BLOCK_END ############

    cv2.imshow('frame',cur_scene)
    time.sleep(0.1)
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
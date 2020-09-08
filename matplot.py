import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

UAV_x = []
ROI_x = []

UAV_speed_2 = []
object_1 = []
object_2 = []
object_3 = []
object_4 = []
object_5 = []

fname = "data.txt"
count = 0
line = 0
n_count = []
with open(fname, 'r') as P:
    for _line in P:
    	count += 1
    	n_count.append(count)
print("Total number of lines is:", count)
# file = open ("data.txt","r")
velocity = open ("velocity.txt","r")
for i in range(count) :
	# txt1 = str(file.readline().replace('\n',''))
	# x = txt1.split()
	# UAV_x.append(int(x[0]))
	# ROI_x.append(int(x[1]))

	txt2 = str(velocity.readline().replace('\n',''))
	y = txt2.split()
	UAV_speed_2.append(float(y[0]))
	object_1.append(float(y[1]))
	object_2.append(float(y[2]))
	object_3.append(float(y[3]))
	object_4.append(float(y[4]))
	object_5.append(float(y[5]))


# plt.plot(n_count,UAV_x,'b-',label = 'UAV 2')
# plt.plot(n_count,ROI_x,'r-',label = 'ROI')

plt.plot(n_count,UAV_speed_2,'b-',label = 'UAV 2')
plt.plot(n_count,object_1,'r-',label = 'object 1')
plt.plot(n_count,object_2,'g-',label = 'object 2')
plt.plot(n_count,object_3,'y-',label = 'object 3')
plt.plot(n_count,object_4,'c-',label = 'object 4')
plt.plot(n_count,object_5,'k-',label = 'object 5')

# for x,y,z in zip(measurement_x,measurement_y,n_measurement):

#     label = "{:.2f}".format(z)

#     plt.annotate(label, # this is the text
#                  (x,y), # this is the point to label
#                  textcoords="offset points", # how to position the text
#                  xytext=(0,10), # distance from text to points (x,y)
#                  ha='center') # horizontal alignment can be left, right or center

# plt.plot(predict_x,predict_y,'r-')
# for x,y,z in zip(predict_x,predict_y,n_predict):

#     label = "{:.2f}".format(z)

#     plt.annotate(label, # this is the text
#                  (x,y), # this is the point to label
#                  textcoords="offset points", # how to position the text
#                  xytext=(0,10), # distance from text to points (x,y)
#                  ha='center') # horizontal alignment can be left, right or center
# plt.plot(measurement_x,measurement_y,'bo',label="Measurement")
# plt.plot(predict_x,predict_y,'ro',label="Predict")
plt.legend(loc='best')
# plt.ylabel('Location(pixel)')
plt.ylabel('Velocity(pixel/deltaT)')
plt.xlabel('Time(deltaT)')
plt.show()
file.close()
velocity.close()



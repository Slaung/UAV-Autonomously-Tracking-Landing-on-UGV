import rospy
import keras
import time
import numpy as np
from keras.models import Model
from keras import backend as K
from keras.layers import Input, Dense, LeakyReLU, Lambda
from sklearn.preprocessing import MinMaxScaler
from std_msgs.msg import String 
from FuzzyLayer import FuzzyLayer
from DefuzzyLayer import DefuzzyLayer
from queue import Queue

### Area queue ###
area_queue = Queue()

### Callback string ###
ros_callback_string = ""
ros_publish_string = ""

def ros_callback(data):
    global ros_callback_string
    ros_callback_string = data.data
    area_queue.put(int(ros_callback_string))

rospy.init_node('neural_network')

### Anaconda and ROS connect ###
ros_sub = rospy.Subscriber("ros_detection_area", String, callback=ros_callback)
ros_pub = rospy.Publisher("anaconda_predict_height", String, queue_size = 10)


sub_rate = rospy.Rate(20)

def data_preprocessing():
    h = []
    area = []

    with open('./data2.txt', 'r') as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split(', ')
            h_value = float(parts[0].split(': ')[1])
            h.append(h_value)
            area_value = int(parts[1].split(': ')[1])
            area.append(area_value)
    return h, area

def FNN_model():
  inp=Input(shape=(1, 1))
  x=FuzzyLayer(32)(inp) 
  x=DefuzzyLayer(1)(x)
  model=Model(inp, x)
  model.summary()
  return model

h, area = data_preprocessing()
model = FNN_model()

model.load_weights('./FNN_best_weight.h5')

minmax = MinMaxScaler()
norm_area = minmax.fit_transform(np.array(area).reshape(-1, 1))
all_h = model.predict(norm_area)

for i in range(len(all_h)):
    print("input_area:", area[i], "real_h:", h[i], "pred_h:", all_h[i][0][0])

count = 0
while(True):
    if(ros_callback_string != ""):

        ### Get queue data ###
        area = area_queue.get()

        ### Predict height ###
        norm_get_area = minmax.transform(np.array(area).reshape(-1, 1))
        predict_height = model.predict(norm_get_area)

        area_queue.task_done()

        ### Publish height to ros ###
        ros_publish_string = str(predict_height[0][0][0])
        ros_pub.publish(ros_publish_string)

        #print("t:", count, ",Anaconda_get_area:", area, ",Predicted_height:", predict_height[0][0][0])
        print("t:", count)
        print("Area:", area)
        print("Pred_h:", predict_height[0][0][0])
        count +=1

    sub_rate.sleep()
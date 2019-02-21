#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt


counter = 0
output = 0
currentAngle = 0

def callback(data):
    global output
    global currentAngle
    rospy.loginfo("I heard %s", data.data)
    if data.data > 400:
        currentAngle = data.data-450
    else:
        output = data.data



if __name__ == '__main__':
    rospy.init_node('visualizer')
    rospy.Subscriber("master_output",Float32,callback)
    rospy.spin()
    
    i = 0


    while i < 1000:
        temp_y = np.random.random()
        x.append(i)
        y.append(temp_y)
        plt.scatter(i, temp_y)
        i += 1
        plt.draw()

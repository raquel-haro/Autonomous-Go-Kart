#!/usr/bin/env python

import Tkinter as tk
import rospy
import random
from std_msgs.msg import Float32

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

def counter_label(label):
    def count():
        global counter
        counter += 1
        if output == -500:
            label.config(text="No Gap!" + "\n\n" + str(round(currentAngle,2)) + "\n\n" + "No Gap!")
        else:
            label.config(text=str(round(output,2)) + "\n\n" + str(round(currentAngle,2)) + "\n\n" + str(round(output,2)-round(currentAngle,2)))
        label.after(10,count)
    count()

if __name__ == '__main__':
    rospy.init_node('visualizer')
    rospy.Subscriber("master_output",Float32,callback)

    root = tk.Tk()
    root.geometry("600x300")
    root.title("Autonomous Vehicle GUI")
    info = tk.Label(root, fg="black",font="Helvetica 30 bold",text="Optimal Angle: \n\nCurrent Wheel Angle:\n\nDifference:",anchor='nw')
    info.pack(side="left")
    info = tk.Label(root, 
                            fg="red",
                            font = "Helvetica 30 bold",
                            anchor='center')
    info.pack(side="right")
    counter_label(info)
    root.mainloop()

    rospy.spin()
    
    


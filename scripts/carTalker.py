#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from random import randint
import random
import json

keyword = "undefined"

def callback(data):
    global keyword 
    if data.data == "armarker"  or data.data == "start" or data.data == "restart":
        keyword = "undefined"
    else:
        keyword = data.data[0:data.data.rfind('.')]


def carTalker():
    pub = rospy.Publisher('carTalker', String, queue_size=5)
    rospy.Subscriber('client_received_key', String, callback)
    rospy.init_node('car_node', anonymous=True)
    rate = rospy.Rate(1/10) # send 1 queuing with 5 packages in 10 seconds        

    while not rospy.is_shutdown():
        data = {
            "velocity":str(randint(1,11)),
            "location":keyword,
            "carConnection":"true"
        }

        pub.publish(json.dumps(data))

        rate.sleep()
        if rospy.is_shutdown():
            # pub.publish("car diconnected")
            rospy.loginfo("car disconnected")

if __name__ == '__main__':
    try:
        carTalker()
    except rospy.ROSInterruptException:
        pass

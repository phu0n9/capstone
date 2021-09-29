#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from random import randint
import random
import json

keyword = "undefined"

def callback(data):
    global keyword 
    if data.data == "armarker" or data.data == "start" or data.data == "restart":
        keyword = "undefined"
    else:
        keyword = data.data[(data.data.rfind('.')+1):(len(data.data))]

def droneTalker():
    pub = rospy.Publisher('droneTalker', String, queue_size=5)
    rospy.Subscriber('client_received_key', String, callback)

    rospy.init_node('drone_node', anonymous=True)
    rate = rospy.Rate(1/10) # send 1 queuing with 5 packages in 10 seconds        

    while not rospy.is_shutdown():
        data = {
            "battery":str(randint(1,101)),
            "altitude":str(randint(1,13)),
            "location":keyword,
            "droneConnection":"true"
        }
        rospy.loginfo(json.dumps(data))
        pub.publish(json.dumps(data))

        rate.sleep()
        if rospy.is_shutdown():
            # pub.publish("car diconnected")
            rospy.loginfo("drone disconnected")

if __name__ == '__main__':
    try:
        droneTalker()
    except rospy.ROSInterruptException:
        pass

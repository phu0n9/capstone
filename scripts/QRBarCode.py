#!/usr/bin/env python

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from std_msgs.msg import String
from pyzbar.pyzbar import decode
import numpy as np
import os
import re
import glob

keyword = ""
rack = ""
state = 1
cap = cv2.VideoCapture('/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0')
cap.set(3,640)
cap.set(4,480)
imgAndScore = {}

class ImageLoader:
  def __init__(self,img_name):
      self.img = cv2.imread(img_name)
      self.__name = img_name
  
  def __str__(self):
      return self.__name

  
def transformStr(data):
  first,second,third = data.split('.')
  if len(first) != 2:
    first = "0"+first
  if len(second) != 2:
    second = "0"+second
  if len(third) != 2:
    third = "0"+third
  return first+"-"+second+"-"+third,first


def callback(data):
  global keyword,rack
  rospy.loginfo("callback %s" % data.data)
  if data.data == "armarker" or data.data == "start" or data.data == "restart":
    keyword = data.data
    rospy.loginfo("callback qrbarcode %s" % keyword)
  else:
    keyword = transformStr(data.data)[0]
    rack = transformStr(data.data)[1]
    rospy.loginfo("callback %s" % keyword)

def getRackNumber(myData):
  rospy.loginfo("getRackNumber %s" %myData[0:myData.find('-')])
  global rack    
  rospy.loginfo("rack %s" %rack)

  if myData[0:myData.find('-')] == rack:
      return False
  return True

def publish_message():
  rospy.init_node('qrbarcode_node', anonymous=True)
  global state,keyword,cap,rack

  myData = ""
  i = 1

  rate = rospy.Rate(10) # 10hz
  sleepRate = rospy.Rate(5)

  while not rospy.is_shutdown():
    success,img = cap.read()

    if state == 1:
      pub = rospy.Publisher('qrbarcode', String, queue_size=5)
      rospy.Subscriber('client_received_key', String, callback)
      rack = ""
      i = 1
      if keyword != "armarker" and keyword != "start" and keyword != "restart":
        state = 2

    elif state == 2:
      imgCopy = img.copy()
      i = 1
      if success == True:
        for barcode in decode(imgCopy):
          myData = ''.join(barcode.data.decode('utf-8').split())[3:]
          rackBarCode = getRackNumber(myData)
          pts = np.array([barcode.polygon],np.int32)
          pts = pts.reshape((-1,1,2))
          cv2.polylines(img,[pts],True,(255,0,255),5)
          pts2 = barcode.rect
          regrex = "^[0-9-]*$"
          rospy.loginfo("myData %s" %myData)
          if rackBarCode == True:
            state = 4
          elif myData != None and len(myData) == 8 and re.match(regrex,myData) and keyword == myData and rackBarCode == False:
            rospy.loginfo("myData %s" %myData)
            state = 3
        cv2.imshow("qrbarcode", img)
        cv2.waitKey(1)
      rate.sleep()

    elif state == 3:
      DIR = '/home/tuan/catkin_ws/src/beginner_tutorials/scripts/picture'
      rack = ""
      if success == True:
        # rospy.loginfo("i %d" %i)
        if i % 6 == 0:
          cv2.imwrite(os.path.join(DIR,str(keyword)+"("+str(i)+")"+'.png'),img)
          rospy.loginfo(str(i)+'.png')
          image = ImageLoader(os.path.join(DIR,str(keyword)+"("+str(i)+")"+'.png'))
          gray = cv2.cvtColor(image.img, cv2.COLOR_BGR2GRAY)
          fm = cv2.Laplacian(gray,cv2.CV_64F).var()
          temp = i
          if (fm < 100):
            i = temp - 1
          else:
            imgAndScore = {str(keyword)+"("+str(i)+")"+'.png':fm}

        elif i == 37:
          # rospy.loginfo("done")
          bestImg = [key for key,value in imgAndScore.items() if value == max(imgAndScore.values())][0]
          [os.remove(cleanUp) for cleanUp in glob.glob("picture/*.png") if not cleanUp.endswith(bestImg)]
          pub.publish(bestImg)
          rack = ""
          rate.sleep()
          state = 1
      
      rate.sleep()
    elif state == 4:
      pub.publish("no rack")
      rate.sleep()
      state = 1
    i = i + 1
    sleepRate.sleep()


if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    cap.release()
    pass
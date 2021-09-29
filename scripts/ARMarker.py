#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import cv2.aruco as aruco

keyword = ""
state = 1
# # AR marker generator: https://chev.me/arucogen/
def callback(data):
  global keyword,state
  keyword = data.data
  if keyword == "armarker":
    state = 2
  elif keyword == "start" or keyword == "restart":
    state = 1
  elif keyword != "start" and keyword != "restart" or keyword != "armarker":
    state = 4

def findArucoMarker(img,markerSize=6,totalMarkers=250,draw=True):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img,bbox)

    return [bbox,ids]

def detectMarkers():
  rospy.init_node('marker_node', anonymous=True)
  
  global keyword,state

  rate = rospy.Rate(10) # 10hz
  sleepRate = rospy.Rate(5)

  cap = cv2.VideoCapture('/dev/v4l/by-id/usb-SuYin_HP_Truevision_HD_HF1016-P82A-HM01-2-REV0101-video-index0')
  cap.set(3,640)
  cap.set(4,480)    

  while not rospy.is_shutdown():   
    success, img = cap.read()
    if state == 1:
      pub = rospy.Publisher('ArMarker', String, queue_size=10)
      rospy.Subscriber('client_received_key', String, callback)
      pub.publish('start')
      rate.sleep()
     
    elif state == 2:
      if success == True:
        arucoFound = findArucoMarker(img)
        if len(arucoFound[0]) != 0:
          for bbox,id in zip(arucoFound[0],arucoFound[1]):
            rospy.loginfo('marker found: %d' % id)
            if type(id) != None:
              pub.publish('marker found')
              state = 3
              rate.sleep()
        cv2.imshow("armarker", img)
        cv2.waitKey(1)
      rate.sleep()
    elif state == 3:
      pub.publish('restart')
      rate.sleep()
    else:
      pub.publish('qrbarcode')
      rate.sleep()

    sleepRate.sleep()

if __name__ == "__main__":
  try:
    detectMarkers()
  except rospy.ROSInterruptException:
    pass

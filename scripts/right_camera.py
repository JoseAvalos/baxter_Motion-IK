#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import cv_bridge
import baxter_interface
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

'''
#yellow
        low_h  = 10
        high_h = 50
        low_s  = 85
        high_s = 175
        low_v  = 70
        high_v = 255

#red
        low_h  = 0
        high_h = 9
        low_s  = 50
        high_s = 255
        low_v  = 50
        high_v = 255
#green
        low_h  = 50
        high_h = 90
        low_s  = 74
        high_s = 147
        low_v  = 160
        high_v = 255

#white
        w_low_h=0
        w_high_h=0
        w_low_s=0
        w_high_s=0
        w_low_v=0
        w_high_v=255
#blue no black
        b_low_h  = 105
        b_high_h = 115
        b_low_s  = 135
        b_high_s = 160
        b_low_v  = 20
        b_high_v = 60
'''

# global obj_found
obj_found = False
# global correct_location 
correct_location = True
#Object color: 0 = green, 1 = blue
obj_color = 0

#Object centroid position in the baxter's stationary base frame 
xb = 0
yb = 0


'''
Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
'''
def callback(message):

    global xb, yb, obj_color, obj_found
    xb = 0
    yb = 0
    #Capturing image of camera
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
    height, width, depth = cv_image.shape
    #print height, width, depth 
    #Converting image to HSV format
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    thresholded = 0
    obj_color = 0
    pub = rospy.Publisher('right_baxter_color', Vector3, queue_size=1)
    rate = rospy.Rate(1000) # 10hz
    #Green colored objects
    if obj_color == 0: 
        low_h  = 10
        high_h = 50
        low_s  = 85
        high_s = 175
        low_v  = 70
        high_v = 255

        thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))
        #Morphological opening (remove small objects from the foreground)
        thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
        thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
        #Morphological closing (fill small holes in the foreground)
        thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
        thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)

        ret,thresh = cv2.threshold(thresholded,147,255,0)

        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        
        cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)

        numobj = len(contours) # number of objects found in current frame
        #print 'Number of objects found in the current frame: ' , numobj
        print(numobj)
        msg=Vector3()
        if numobj > 0:
            print("here")
            moms = cv2.moments(contours[0])
            #print moms
            if moms['m00']>20:
                cx = int(moms['m10']/moms['m00'])
                cy = int(moms['m01']/moms['m00'])

                xb = -(cy - (height/2))*.0024*.429 + .59 -0.015
                yb = -(cx - (width/2))*.0024*.429 - .40  + 0.025



                msg.x=xb
                msg.y=yb
                msg.z=0
                
                pub.publish(msg)
                print("xb: ",xb,"/n")
                print("yb: ",yb,"/n")

            #print "Found blue ", numobj,  "object(s)" 
            obj_found = True
    else:
    	print ("Couldn't find any green or blue objects.")
     
    cv2.imshow("Original", cv_image)
    cv2.imshow("Thresholded", thresholded)
    cv2.waitKey(3)


def main():
    #Initiate left hand camera object detection node
    rospy.init_node('right_camera')
    #Create names for OpenCV images and orient them appropriately
    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Original", 1200,800)
    cv2.namedWindow("Thresholded", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Thresholded", 1200,800)
    rospy.Subscriber("cameras/right_hand_camera/image", Image, callback)
    rospy.spin()


if __name__ == '__main__':
     main()

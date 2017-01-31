#!/usr/bin/env python
import argparse
import struct
import sys
import threading 
from geometry_msgs.msg import Vector3
import rospy
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndpointState
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

class InfoClass_1(object):
    def __init__(self):
        self._event=threading.Event()
        self._msg=None

    def __call__(self,msg):
        self._msg=msg
        self._event.set()

    def get_msg(self, timeout=None):
        self._event.wait(timeout)
        return self._msg  

class InfoClass_2(object):
    def __init__(self):
        self._event=threading.Event()
        self._msg=None

    def __call__(self,msg):
        self._msg=msg
        self._event.set()

    def get_msg(self, timeout=None):
        self._event.wait(timeout)
        return self._msg  

class InfoClass_3(object):
    def __init__(self):
        self._event=threading.Event()
        self._msg=None

    def __call__(self,msg):
        self._msg=msg
        self._event.set()

    def get_msg(self, timeout=None):
        self._event.wait(timeout)
        return self._msg  

class InfoClass_4(object):
    def __init__(self):
        self._event=threading.Event()
        self._msg=None

    def __call__(self,msg):
        self._msg=msg
        self._event.set()

    def get_msg(self, timeout=None):
        self._event.wait(timeout)
        return self._msg  

def initial_position(k):
    print "Going to initial position"
    pub_initial = rospy.Publisher('left_position_baxter', Vector3, queue_size=1)
    value_1= InfoClass_1()
    rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, value_1)
    envio_initial=Vector3()
    envio_initial.x=50
    envio_initial.y=-1
    envio_initial.z=-1
    while(k>0):      
        position_1=value_1.get_msg()
        pub_initial.publish(envio_initial)
 
        if  position_1.pose.position.z<0.15 and \
            position_1.pose.position.z>0.05 and \
            position_1.pose.position.y<0.4 and \
            position_1.pose.position.y>0.3 and \
            position_1.pose.position.x<0.7 and \
            position_1.pose.position.x>0.5 :
            k=k-1
            if k%1000==0:
                print(k)
 
def get_point(k):
    print "Finding point"
    position_x=0
    position_y=0
    media=k
    while (k>0) :
        value_1= InfoClass_1()
        rospy.Subscriber("left_baxter_color",Vector3, value_1)
        position_color=value_1.get_msg()
        position_x+=position_color.x
        position_y+=position_color.y
        k-=1

    position_x=position_x/media
    position_y=position_y/media

    print "Point found: "
    return position_x, position_y 
    print position_x
    print position_y

def go_position(k,z):

    pub_go = rospy.Publisher('robot/limb/left/joint_command', JointCommand, queue_size=1)
    
    my_position_x, my_position_y = get_point(10)
    x_send=round(my_position_x,4)
    print x_send
    y_send=round(my_position_y,4)
    print y_send
    wait=k

    value_2= InfoClass_2()
    rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, value_2)
    pub_near = rospy.Publisher('left_position_baxter', Vector3, queue_size=1)

    envio_near=Vector3()
    envio_near.x=x_send
    envio_near.y=y_send
    envio_near.z=z

    while(wait>0):    
        pub_near.publish(envio_near)
        position_near=value_2.get_msg()
        #print wait
        if  position_near.pose.position.z<z-0.35 and \
            position_near.pose.position.z>z-0.47 and \
            position_near.pose.position.y>y_send-0.05 and \
            position_near.pose.position.y<y_send+0.05 and \
            position_near.pose.position.x<x_send+0.05 and \
            position_near.pose.position.x>x_send-0.05 :
            wait=wait-2
            if wait%1000==0:
                print(wait)
 
    wait=k*1.05
    print("descenso...")
    
    envio_near.x=x_send
    print wait
    l=0.105
    variation=l/wait
    print variation
    print "Robot over the object"

    while(wait>0):
        for x in range (0,30):
            envio_near.z=z
        pub_near.publish(envio_near)
        wait=wait-1
        z=z-variation
    print "Robot in the object"
    

def go_box(k):
    wait=k
    print "Moving to the box"
    value_3= InfoClass_3()
    rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, value_3)
    pub_box = rospy.Publisher('left_position_baxter', Vector3, queue_size=1)
    position_actual=value_3.get_msg()
    print position_actual
    position_better= Vector3()
    position_better.x=position_actual.pose.position.x+0.1
    position_better.y=position_actual.pose.position.y-0.1
    if (position_better.x>0.60):
        position_better.x-0.75
    position_better.z=position_actual.pose.position.z+0.7

    print "Init up"
    print position_better
    for x in range (0,3000):
        pub_box.publish(position_better) 
    print "Finish up"
    x_send=1000
    envio_box=Vector3()
    envio_box.x=x_send
    envio_box.y=0
    envio_box.z=0
    while(wait>0):
        pub_box.publish(envio_box) 
        position_near=value_3.get_msg()
        if  position_near.pose.position.y<0.20 and \
            position_near.pose.position.z>-0.045:
            wait=wait-2
            if wait%1000==0:
                print(wait)
    print "Robot on box"
    
def main():
    factor=9

    print("Left Init node")
    rospy.init_node("Left_Ik_service_client", anonymous=True)
    rate= rospy.Rate(1000)

    left_gripper=Gripper('left')
    left_gripper.calibrate()
    left_gripper.open()
    while not rospy.is_shutdown():
            
        initial_position(round(6000*factor))#10ms
        go_position(round(10000*factor),0.25)
        for x in range (0,int(round(2500*factor))):
            left_gripper.close()
            #initial_position(10000)#3ms
        go_box(round(250*factor))
        for x in range (0,int(round(800*factor))):
            left_gripper.open()
        print "Objecto dejado"

'''
        
        initial_position(10000)#3ms
             
'''


if __name__ == '__main__':
   
    try:
        main()
    except Exception:
        pass

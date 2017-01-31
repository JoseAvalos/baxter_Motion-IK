#!/usr/bin/env python 
import pygame
import rospy
import time
from geometry_msgs.msg import Vector3
import argparse
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION
delta=0.2
class MainWindow(object):

    def __init__(self):
        pygame.init()
        rospy.init_node('node_mouse', anonymous=True)
        right_gripper=Gripper('right')
        left_gripper=Gripper('left')
        #limb_0=baxter_interface.Limb('right')
        #limb_0.set_joint_positions({'right_w2': 0.00})

        def set_w2():
            limb=baxter_interface.Limb('left')
            current_position = limb.joint_angle('left_w2')
            #send=current_position+delta
            #if(send>2.80 or send<-2.80):
            #    delta=-delta
            #    time.sleep(0.15)
            #print send
            return current_position
            #joint_command = {joint_name: send}
            #limb.set_joint_positions(joint_command)
            #if(current_position-send>0):
            #    delta=-1
        def set_s1():
            limb=baxter_interface.Limb('left')
            current_position = limb.joint_angle('left_s1')
            #send=current_position+delta
            #if(send>2.80 or send<-2.80):
            #    delta=-delta
            #    time.sleep(0.15)
            #print send
            return current_position 
        def set_e1():
            limb=baxter_interface.Limb('left')
            current_position = limb.joint_angle('left_e1')
            #send=current_position+delta
            #if(send>2.80 or send<-2.80):
            #    delta=-delta
            #    time.sleep(0.15)
            #print send
            return current_position       
            
            
        pygame.display.set_caption('Game')
        pygame.mouse.set_visible(True)
        vec = Vector3(1, 2, 3)
        right_gripper.calibrate()
        left_gripper.calibrate()
        try:
            pub = rospy.Publisher('mouse', Vector3, queue_size=1)
            rate = rospy.Rate(1000) # 10hz
            screen = pygame.display.set_mode((640,480), 0, 32)
            pygame.mixer.init()
            a=0 ,0 ,0
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEBUTTONUP:
                        a = pygame.mouse.get_pressed()
                print a
                weq=0
                if a[0]==1:
                    left_gripper.open()
                else:
                    left_gripper.close()
                if a[2]==1:
                    right_gripper.open()
                else:
                    right_gripper.close()

                vec.x=set_w2()
                vec.y=a[1]
                vec.z=0
                print vec
                pub.publish(vec)
            pygame.mixer.quit()
            pygame.quit()

        except rospy.ROSInterruptException:
            pass
MainWindow()
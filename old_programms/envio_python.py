#!/usr/bin/env python
""" 
prueba_01 
"""
import argparse
# rospy - ROS Python API
import rospy
# baxter_interface - Baxter Python API
import baxter_interface
# initialize our ROS node, registering it with the Master

# Import the necessary Python modules
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def callback(data):
	rospy.loginfo( "I heard")
   	global Name
   	global Position 
   	Name = data.name
   	Position = data.position

def hacer():
	rospy.init_node('n_envioBaxter', anonymous=True)
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	rospy.Subscriber("t_sendbaxter", JointState, callback)
	#rospy.spin

	# create an instance of baxter_interface's Limb class
	limb_right = baxter_interface.Limb2('right')
	limb_left = baxter_interface.Limb2('left')

	# get the right limb's current joint angles
	angles_right = limb_right.joint_angles()
	angles_left = limb_left.joint_angles()

	print(Name[1])
	# print the current joint angles
	#print angles

	# reassign new joint angles (all zeros) which we will later command to the limb
	#angles['right_s0']=0.5
	#angles['right_s1']=0.5
	#angles['right_e0']=0.5
	#angles['right_e1']=0.5
	#angles['right_w0']=0.5
	#angles['right_w1']=0.5
	#angles['right_w2']=0.5

	# print the joint angle command


	# move the right arm to those joint angles

	# Baxter wants to say hello, let's wave the arm

	# store the first wave position 
	wave_right_1 = {'right_s0': Position[8], 'right_s1': Position[9], 'right_e0': Position[10], 'right_e1': Position[11], 'right_w0': Position[12], 'right_w1': Position[13], 'right_w2': Position[14]}
	wave_left_1 = {'left_s0': Position[1], 'left_s1':Position[2], 'left_e0': Position[3], 'left_e1': Position[4], 'left_w0': Position[5], 'left_w1': Position[6], 'left_w2': Position[7]}

	#wave_right_2 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 0.607, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
	# store the second wave position
	#wave_left_2 = {'left_s0': -0.395, 'left_s1': -0.202, 'left_e0': 1.031, 'left_e1': 1.981, 'left_w0': -1.979, 'left_w1': -1.100, 'left_w2': -0.448}

	#for x in range(0, 1000):
	angles_left=wave_left_1
	angles_right=wave_right_1
	#print angles
	limb_left.move_to_joint_positions(angles_left)
	limb_right.move_to_joint_positions(angles_right)
	#angles_left=wave_left_2
	#angles_right=wave_right_2
	#print angles
	#limb_left.move_to_joint_positions(angles_left)
	#limb_right.move_to_joint_positions(angles_right)

	# quit
	#quit()

if __name__ == '__main__':
	i = 1
	while i <= 10:
		hacer()
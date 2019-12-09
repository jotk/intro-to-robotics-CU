# Sample program to solve for the Inverse Kinematics of a particular hand position and send the robot arm there

import rospy
import sys
import intera_interface
from sawyer_pykdl import sawyer_kinematics

# Global Variables

def move_position(xs,ys,zs):
	limb = intera_interface.Limb('right')
	kinematics = sawyer_kinematics('right')
	xyz_pos = [xs, ys, zs]
	x = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos)))
	limb.move_to_joint_positions(x)
	rospy.sleep(1.0)

def gripper_open():
	gripper=intera_interface.Gripper()
	gripper.open()
	rospy.sleep(0.5)

def gripper_close():
	gripper=intera_interface.Gripper()
	gripper.close()
	rospy.sleep(0.5)

def main():
	rospy.init_node("test_ik")

	gripper_close()
	move_position(0.5,0.033,0.014)
	gripper_open()
	move_position(0.0,0.033,0.2)
	gripper_close()
	

if __name__ == '__main__':
	sys.exit(main())

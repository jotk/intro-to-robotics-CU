import intera_interface
import rospy
import copy 
import std_msgs

import pdb
import pickle
import random
import cv2 # You may have to "pip install opencv-contrib-python" to install this
import numpy as np # You may have to "pip install numpy" to install this

from geometry_msgs.msg import Pose, Point, Quaternion
from loca import add_color_range_to_detect,check_if_color_in_range,do_color_filtering,expand,expand_nr,get_blobs,get_blob_centroids, print_fun

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None

g_gripper = None
g_sub_open = None
g_sub_close = None
g_is_pneumatic = False

hover_dist = 0.203
table_level = -0.06
lego_height = 0.018
lego_base_height = -0.03
stack_height = lego_base_height





def init():
    global g_limb, g_orientation_hand_down, g_position_neutral
    rospy.init_node('Ivan_Jot_Albert_Angus')
    g_limb = intera_interface.Limb('right')

    # This quaternion will have the hand face straight down (ideal for picking tasks)
    g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y =0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073

    # This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
    g_position_neutral = Point()
    g_position_neutral.x = 0.449559195663
    g_position_neutral.y = 0.16070379419
    g_position_neutral.z = 0.212938808947


def gripper_open():
	gripper=intera_interface.Gripper()
	gripper.open()
	rospy.sleep(0.5)

def gripper_close():
	gripper=intera_interface.Gripper()
	gripper.close()
	rospy.sleep(0.5)

def move_arm(pos):
    print(pos.position.x,pos.position.y,pos.position.z)
    target_joint_angles = g_limb.ik_request(pos, "right_hand")
    if target_joint_angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(pos))
        return
    g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
    #new_hand_pose = copy.deepcopy(g_limb.tip_states.states[0].pose)
    #new_angles = g_limb.joint_angles()
    #new_hand_pose = g_limb._right_hand_pose
    #print("New hand pose:\n %s" % str(new_hand_pose))
    #print("New joint angles:\n %s" % str(new_angles))


def get_lego(x,y):
    global table_level, hover_dist

    gripper_open()

    target_pose = Pose()
    target_pose.orientation = copy.deepcopy(g_orientation_hand_down)

    target_pose.position.x = x # Add 20cm to the x axis position of the hand	
    target_pose.position.y = y	
    target_pose.position.z = hover_dist
    move_arm(target_pose)

    target_pose.position.z = table_level
    move_arm(target_pose)

    gripper_close()
    
    target_pose.position.z = hover_dist
    move_arm(target_pose)

def to_target(x,y):
    global stack_height, lego_height, hover_dist, lego_base_height

    target_pose = Pose()
    target_pose.orientation = copy.deepcopy(g_orientation_hand_down)

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = hover_dist
    move_arm(target_pose)

    target_pose.position.z = stack_height
    move_arm(target_pose)
    stack_height += lego_height

    gripper_open()
    rospy.sleep(0.5)
    g_limb.move_to_neutral()


def main():
    global g_limb, g_position_neutral, g_orientation_hand_down

    #Import homwwork4 for location
    img = cv2.imread('./color-blobs.png')
    add_color_range_to_detect([0,0,200], [0,0,255]) # Detect red
    add_color_range_to_detect([0,200,0], [0,255,0]) # Detect green
    add_color_range_to_detect([200,0,0], [255,0,0]) # Detect blue
    img_mask = do_color_filtering(img)
    blobs = get_blobs(img_mask)
    object_positions_list = get_blob_centroids(blobs)
    img_markup = img.copy()
    for obj_pos in object_positions_list:
    	obj_pos_vector = np.array(obj_pos).astype(np.int32) # In case your object positions weren't numpy arrays
    	img_markup = cv2.circle(img_markup,(obj_pos_vector[1], obj_pos_vector[0]),5,(0,0,0),10)
    	print("Object pos: " + str(obj_pos_vector))


    # Initialize gripper and sawyer arm
    init()

    # Move the arm to its neutral position
    g_limb.move_to_neutral()

    #rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
    #rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))

    # Create a new pose (Position and Orientation) to solve for
    #target_pose = Pose()
    #target_pose.position = copy.deepcopy(g_position_neutral)
    #target_pose.orientation = copy.deepcopy(g_orientation_hand_down)

    #target_pose.position.x += 0.2 # Add 20cm to the x axis position of the hand
    #target_pose.position.z -= 0.2656
    
    # Call the IK service to solve for joint angles for the desired pose
    #target_joint_angles = g_limb.ik_request(target_pose, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.3)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish

    #get_lego(0.617,0.305)
    #to_target(0.607,-0.153)

    get_lego(0.617,0.305)
    to_target(0.607,-0.153)


    #get_lego(0.3,0.2656)
    #to_target(0.2,0.5,0.2056)
    #g_limb.move_to_neutral()
    # Find the new coordinates of the hand and the angles the motors are currently at
    new_hand_pose = copy.deepcopy(g_limb._tip_states.states[0].pose)
    new_angles = g_limb.joint_angles()
    #rospy.loginfo("New Hand Pose:\n %s" % str(new_hand_pose))
    #rospy.loginfo("Target Joint Angles:\n %s" % str(target_joint_angles))
    #rospy.loginfo("New Joint Angles:\n %s" % str(new_angles))


if __name__ == "__main__":
    main()

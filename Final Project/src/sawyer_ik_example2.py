import intera_interface
import rospy
import copy 
import std_msgs
import camera_image_original
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Point, Quaternion


class robot():
    def __init__(self, hover_dist = 0.213, table_level = -0.06, lego_height = 0.018, lego_base_height = 0.0, joint_speed=.3):
        self._limb = None
        self._orientation_hand_down = None
        self._position_neutral = None
        self._gripper = None
        self._sub_open = None
        self._sub_close = None
        self._is_pneumatic = False

        self.next_lego_target = None
        self.legos_stacked_num = 0
        self.base = None

        self.hover_dist = hover_dist
        self.table_level = table_level
        self.lego_height = lego_height
        self.lego_base_height = lego_base_height
        self.stack_height = lego_base_height - .05
        self._arm_camera_joint_angles = {'right_j6': 4.7126953125, 'right_j5': 0.30809765625, 'right_j4': 1.6166796875, 'right_j3': 1.5452109375, 'right_j2': -1.8612998046875, 'right_j1': -0.1184775390625, 'right_j0': 0.424828125}

        rospy.init_node('Ivan_Jot_Albert_Angus')
        self._limb = intera_interface.Limb('right')
        self._gripper = intera_interface.Gripper()
        self._limb.set_joint_position_speed(joint_speed)
        self._cam = None
        self.lego_locations = None

        # This quaternion will have the hand face straight down (ideal for picking tasks)
        self._orientation_hand_down = Quaternion()
        self._orientation_hand_down.x = 0.704238785359
        self._orientation_hand_down.y = 0.709956638597
        self._orientation_hand_down.z = -0.00229009932359
        self._orientation_hand_down.w = 0.00201493272073

        # This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
        self._position_neutral = Point()
        self._position_neutral.x = 0.449559195663
        self._position_neutral.y = 0.16070379419
        self._position_neutral.z = 0.212938808947

        print("Waking up...")
        self._limb.move_to_neutral()
        rospy.sleep(2)
        print("Moved to nuetral.")

    def init_tester_base(self):
        self.base = [[ 0.626546765216, -0.167908411913], [0.604467187097, -0.147199806627]]
        # change_vector = [self.base[0][0] - self.base[1][0], self.base[0][1] - self.base[0][1]]
        # self.base.append([.607127122,-0.09970412294])
        # self.base.append([0.6001588995,-0.05699326512])
        self.next_lego_target = self.base[0]


    def take_image(self):
        #move to pictuyre pos
        print("About to take a picture...")
        self._limb.move_to_joint_positions(self._arm_camera_joint_angles)
        rospy.sleep(2)
        camera_image_original.main()
        rospy.sleep(2)

        self.lego_locations = camera_image_original.get_coords_of_legos()
        print("Picture taken and analyzed!")
        # camera_image.main()
        # g_image = camera_image.cv_image
        # camera_image.clean_shutdown()
        # '''
        # head_display = intera_interface.HeadDisplay()
        # head_display.display_image("/home/abcd3302/ros_ws/a.png")
        # '''
    def go_to_nuetral(self):
        self._limb.move_to_neutral()
    def open_grip(self, msg):
        rospy.loginfo("Opening gripper")
        if self._is_pneumatic:
            self._gripper.set_ee_signal_value('grip', False)
        else:
            self._gripper.open()

    def close_grip(self, msg):
        rospy.loginfo("Closing gripper")
        if self._is_pneumatic:
            self._gripper.set_ee_signal_value('grip', True)
        else:
            self._gripper.close()

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(0.5)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(0.5)

    def move_arm(self, pos):
        # print(pos.position.x,pos.position.y,pos.position.z)
        target_joint_angles = self._limb.ik_request(pos, "right_hand")
        if target_joint_angles is False:
            rospy.logerr("Couldn't solve for position %s" % str(pos))
            return
        self._limb.move_to_joint_positions(target_joint_angles, timeout=2)


    def get_lego(self, x,y):
        self.gripper_open()

        target_pose = Pose()
        target_pose.orientation = copy.deepcopy(self._orientation_hand_down)
        print(x, y)
        target_pose.position.x = x # Add 20cm to the x axis position of the hand
        target_pose.position.y = y
        target_pose.position.z = self.hover_dist
        self.move_arm(target_pose)

        target_pose.position.z = self.table_level + .02
        self.move_arm(target_pose)

        self.gripper_close()

        target_pose.position.z = self.hover_dist
        self.move_arm(target_pose)

    def to_target(self, x,y):
        global stack_height, lego_height, hover_dist, lego_base_height

        target_pose = Pose()
        target_pose.orientation = copy.deepcopy(self._orientation_hand_down)

        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = self.hover_dist
        self.move_arm(target_pose)

        target_pose.position.z = self.stack_height
        self.move_arm(target_pose)
        self.stack_height += self.lego_height
        self.gripper_open()

    def init_tester_coords(self):
        """
        Used so we can hardcode a configuration for the lego peices for testing and such
        """
        self.lego_locations = [[0.798089317137,0.381599983921], [0.795273296797, 0.349632885306]]
        # find vector difference

    def stack(self, coord):
        self.get_lego(coord[0], coord[1])
        self.to_target(self.next_lego_target[0], self.next_lego_target[1])
        self.change_next_target()
        self.go_to_nuetral()

    def change_next_target(self):
        """
        Will need to change if the target structure changes
        """
        if self.legos_stacked_num > 1: # should be 3
            self.stack_height = self.stack_heigh + lego_base_height # changes the stack height
            self.next_lego_target = self.base[0]
        else:
            self.next_lego_target = self.base[(self.legos_stacked_num+4)%4 + 1] # go to the coord in base which is next to the pose now
        self.legos_stacked_num = self.legos_stacked_num + 1




def main():
    # Initialize sawyer arm
    sawyerArm = robot()
    # sawyerArm.take_image()
    sawyerArm.init_tester_coords()
    sawyerArm.init_tester_base()
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


    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish

    # get_lego(0.617,0.345)
    for lego_coord in sawyerArm.lego_locations:
        sawyerArm.stack(lego_coord)
    # sawyerArm.get_lego(0.792659329811,0.388437401831)
    # sawyerArm.to_target(0.607,-0.153)
    

    #get_lego(0.3,0.2656)
    #to_target(0.2,0.5,0.2056)
    #g_limb.move_to_neutral()
    # Find the new coordinates of the hand and the angles the motors are currently at

    new_hand_pose = copy.deepcopy(sawyerArm._limb._tip_states.states[0].pose)
    new_angles = sawyerArm._limb.joint_angles()

    #rospy.loginfo("New Hand Pose:\n %s" % str(new_hand_pose))
    #rospy.loginfo("Target Joint Angles:\n %s" % str(target_joint_angles))
    #rospy.loginfo("New Joint Angles:\n %s" % str(new_angles))

if __name__ == "__main__":
    main()

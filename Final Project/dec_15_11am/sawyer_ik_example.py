import intera_interface
import rospy
import copy 
import std_msgs


from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None

g_gripper = None
g_sub_open = None
g_sub_close = None
g_is_pneumatic = False

def initGripper():
    global g_sub_open, g_sub_close, g_gripper, g_is_pneumatic

    g_sub_open = rospy.Subscriber('/cairo/sawyer_gripper_open', std_msgs.msg.Empty, open_grip)
    g_sub_close = rospy.Subscriber('/cairo/sawyer_gripper_close', std_msgs.msg.Empty, close_grip)

    g_gripper = intera_interface.get_current_gripper_interface()
    g_is_pneumatic = isinstance(g_gripper, intera_interface.SimpleClickSmartGripper)

    if g_is_pneumatic:
        if g_gripper.needs_init():
            g_gripper.initialize()
            if g_gripper.needs_init(): return False
    else:
        if not g_gripper.is_calibrated():
            g_gripper.calibrate()
            if not g_gripper.is_calibrated(): return False

    return True


def init():
    global g_limb, g_orientation_hand_down, g_position_neutral
    rospy.init_node('cairo_sawyer_ik_example')
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

def open_grip(msg):
    global g_gripper, g_is_pneumatic
    rospy.loginfo("Opening gripper")
    if g_is_pneumatic:
        g_gripper.set_ee_signal_value('grip', False)
    else:
        g_gripper.open()

def close_grip(msg):
    global g_gripper, g_is_pneumatic

    rospy.loginfo("Closing gripper")
    if g_is_pneumatic:
        g_gripper.set_ee_signal_value('grip', True)
    else:
        g_gripper.close()

def gripper_open():
	gripper=intera_interface.Gripper()
	gripper.open()
	rospy.sleep(0.5)

def gripper_close():
	gripper=intera_interface.Gripper()
	gripper.close()
	rospy.sleep(0.5)

def get_lego(x,z):
	gripper_open()	
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
    	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)

	target_pose.position.x += x # Add 20cm to the x axis position of the hand	
	target_pose.position.y -=0.1	
	target_pose.position.z -= z
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return

        g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
    	gripper_close()
        g_limb.move_to_neutral()

def to_target(x,y,z):
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
    	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)

	target_pose.position.x += x # Add 20cm to the x axis position of the hand
	target_pose.position.y -= y
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)    	
	
	target_pose.position.z -= z
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)    	
	#target_pose.position.z += 0.2656 # Add 20cm to the x axis position of the hand
	#target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	#if target_joint_angles is False:
	#	rospy.logerr("Couldn't solve for position %s" % str(target_pose))
	#	return
	#g_limb.move_to_joint_positions(target_joint_angles, timeout=2)    
	gripper_open()

def main():
    global g_limb, g_position_neutral, g_orientation_hand_down
    init()
    #initGripper()
    # Move the arm to its neutral position
    g_limb.move_to_neutral()

    rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
    rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))

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

    get_lego(0.2,0.2656)
    to_target(0.2,0.5,0.2356)
    g_limb.move_to_neutral()
    #get_lego(0.3,0.2656)
    #to_target(0.2,0.5,0.2056)
    #g_limb.move_to_neutral()
    # Find the new coordinates of the hand and the angles the motors are currently at
    new_hand_pose = copy.deepcopy(g_limb._tip_states.states[0].pose)
    new_angles = g_limb.joint_angles()
    rospy.loginfo("New Hand Pose:\n %s" % str(new_hand_pose))
    rospy.loginfo("Target Joint Angles:\n %s" % str(target_joint_angles))
    rospy.loginfo("New Joint Angles:\n %s" % str(new_angles))


if __name__ == "__main__":
    main()

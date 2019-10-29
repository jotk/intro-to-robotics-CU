import rospy
import json
import copy
import time
import math
import sys
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
SERVO_ANGLE = math.radians(80)
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
IR_sensors = [0,0,0,0,0]
PING_DIST = 0.0
#TODO: Create data structure to hold map representation

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
publisher_render = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting track. 
CYCLE_TIME = 0.05 # In seconds
X_RESOLUTION = 0.036 # meters per pixel ((1200/#squares)*0.0015)
Y_RESOLUTION = 0.03428571 # meters per pixel ((800/#squares)*0.0015)
M_PI = 3.1415927

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
    global subscriber_odometry, subscriber_state
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry, IR_sensors, world_map

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        begin_time = time.time()

        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        motor_msg = Float32MultiArray()
    	subscriber_state
    	if (IR_sensors[1] < IR_THRESHOLD):
       		motor_msg.data = [-1.0,1.0]
    	elif (IR_sensors[3] < IR_THRESHOLD):
       		motor_msg.data = [1.0,-1.0]
    	else:
       		motor_msg.data = [1.0,1.0]

    	#TODO: Implement loop closure here
        if IR_sensors[1] < IR_THRESHOLD and IR_sensors[2] < IR_THRESHOLD and IR_sensors[3] < IR_THRESHOLD:
        	motor_msg.data = [1.0,1.0]
        	#rospy.loginfo("Loop Closure Triggered Loop Closure Triggered Loop Closure Triggered")

       	publisher_ping.publish()
    	publisher_motor.publish(motor_msg)
    	subscriber_odometry
        publisher_render.publish()

        #rospy.loginfo("X: %f  Y: %f  Theta: %f", pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta)
        #rospy.loginfo("x_robot: %f  x_obj: %f  y_robot: %f  y_obj: %f", pose2d_sparki_odometry.x, x_world, pose2d_sparki_odometry.y, y_world)

        populate_map_from_ping()
        #display_map()

        #TODO: Implement CYCLE TIME
        end_time = time.time()
        time_elapsed = (end_time-begin_time)
        if (time_elapsed < 0.05):
        	rospy.sleep(CYCLE_TIME-time_elapsed)


def init():
	global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_render
	global subscriber_odometry, subscriber_state
	global pose2d_sparki_odometry, CYCLE_TIME, world_map, visual_map

    #TODO: Set up your publishers and subscribers
	rospy.init_node('underdogs', anonymous=True)

	publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
	publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
	publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10 )
	publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
	publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)

	subscriber_odometry = rospy.Subscriber("/sparki/odometry", Pose2D, callback_update_odometry)
	subscriber_state = rospy.Subscriber("/sparki/state", String, callback_update_state)

	#TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
	pose2d_sparki_odometry = Pose2D()
	pose2d_sparki_odometry.x = 0.0
	pose2d_sparki_odometry.y = 0.0
	pose2d_sparki_odometry.theta = 0.0
	publisher_odom.publish(pose2d_sparki_odometry)
	rospy.sleep(CYCLE_TIME*3)

    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
	servoMsg = Int16()
	servoMsg.data = int(80)
	publisher_servo.publish(servoMsg)
	rospy.sleep(CYCLE_TIME*3)
	#INITIALIZE BOOLEAN ARRAY FOR world_map variable
	world_map = [[True]*35 for _ in range(50)]
	visual_map = [[1]*35 for _ in range(50)]

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    #TODO: Copy this data into your local odometry variable
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta

def callback_update_state(data):
    global IR_sensors, PING_DIST
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables

    IR_sensors[0] = state_dict['light_sensors'][0]
    IR_sensors[1] = state_dict['light_sensors'][1]
    IR_sensors[2] = state_dict['light_sensors'][2]
    IR_sensors[3] = state_dict['light_sensors'][3]
    IR_sensors[4] = state_dict['light_sensors'][4]

    try:
    	PING_DIST = state_dict['ping']
    except:
    	PING_DIST = -1

    #rospy.loginfo("%f", PING_DIST)

def convert_ultrasonic_to_robot_coords(x_us):
	global SERVO_ANGLE, pose2d_sparki_odometry

    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    #x_r, y_r = 0., 0.
	x_r = math.cos(SERVO_ANGLE+pose2d_sparki_odometry.theta)*x_us
	y_r = math.sin(SERVO_ANGLE+pose2d_sparki_odometry.theta)*x_us

	return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
	global pose2d_sparki_odometry
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    #x_w, y_w = 0., 0.
	x_w = pose2d_sparki_odometry.x+x_r
	y_w = pose2d_sparki_odometry.y+y_r
	return x_w, y_w

def populate_map_from_ping():
	global world_map, PING_DIST, pose2d_sparki_odometry

	#converting to world coordinates of the object
	if (PING_DIST != -1):
		x_robot, y_robot = convert_ultrasonic_to_robot_coords(PING_DIST)
		x_world, y_world = convert_robot_coords_to_world(x_robot, y_robot)
		x_cell,y_cell,cell_index = ij_to_cell_index(x_world,y_world)
		x_robot_cell,y_robot_cell,cell_index_robot = ij_to_cell_index(pose2d_sparki_odometry.x,pose2d_sparki_odometry.y)
		#x_center,y_center = cell_index_to_ij(cell_index)
		if (world_map[x_cell][y_cell] != False):
			world_map[x_cell][y_cell] = False
			print('X-obstacle-cell: ' + str(x_cell) + '  Y-obstacle-cell: ' + str(y_cell))
			visual_map[x_cell][y_cell] = 0
			display_map()
			a,b,c = ij_to_cell_index(x_world,y_world)
			d,e,f = ij_to_cell_index(0.035,0.033)
			g = cost(f,c)

		if (world_map[x_robot_cell][y_robot_cell] != False):
			world_map[x_robot_cell][y_robot_cell] = False
			visual_map[x_robot_cell][y_robot_cell] = 'A' #track robot's trail
			print('X-robot-cell: ' + str(x_robot_cell) + '  Y-robot-cell: ' + str(y_robot_cell))
			display_map()
			a,b,c = ij_to_cell_index(pose2d_sparki_odometry.x,pose2d_sparki_odometry.y)
			d,e,f = ij_to_cell_index(0.035,0.033)
			g = cost(f,c)
	else:
		x_robot_cell,y_robot_cell,cell_index_robot = ij_to_cell_index(pose2d_sparki_odometry.x,pose2d_sparki_odometry.y)
		if (world_map[x_robot_cell][y_robot_cell] != False):
			world_map[x_robot_cell][y_robot_cell] = False
			visual_map[x_robot_cell][y_robot_cell] = 'A' #track robot's trail
			print('X-robot-cell: ' + str(x_robot_cell) + '  Y-robot-cell: ' + str(y_robot_cell))
			display_map()
			a,b,c = ij_to_cell_index(pose2d_sparki_odometry.x,pose2d_sparki_odometry.y)
			d,e,f = ij_to_cell_index(0.035,0.033)
			g = cost(f,c)


def display_map():
	global visual_map
	#TODO: Display the map
	for y in range(0,35):
		for x in range(0,50):
			string_type = str(visual_map[x][34-y])
			sys.stdout.write(string_type + " ")
		sys.stdout.write("\n")
		sys.stdout.flush()

def ij_to_cell_index(i,j):
	global X_RESOLUTION, Y_RESOLUTION

	#TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
	x_cell = math.floor(i/X_RESOLUTION)
	y_cell = math.floor(j/Y_RESOLUTION)

	#count up vertically (along y-axis)
	grid_int = (x_cell*35) + y_cell
	return int(x_cell),int(y_cell),grid_int

def cell_index_to_ij(cell_index):
	global X_RESOLUTION, Y_RESOLUTION

	#TODO: Convert from cell_index to (i,j) center coordinates
	y_cell_center = ((cell_index % 35)*Y_RESOLUTION)+(Y_RESOLUTION/2)
	x_cell_center = (((cell_index-y_cell_center)/35)*X_RESOLUTION)+(X_RESOLUTION/2)
	return x_cell_center, y_cell_center

def cost(cell_index_from, cell_index_to):
	global visual_map
	#TODO: Return cost of traversing from one cell to another
	x1_center_coord, y1_center_coord = cell_index_to_ij(cell_index_from)
	x2_center_coord, y2_center_coord = cell_index_to_ij(cell_index_to)
	x1_cell, y1_cell, cell1 = ij_to_cell_index(x1_center_coord,y1_center_coord)
	x2_cell, y2_cell, cell2 = ij_to_cell_index(x2_center_coord,y2_center_coord)

	x_difference = x2_cell - x1_cell
	y_difference = y2_cell - y1_cell
	min_difference = abs(x_difference) + abs(y_difference)
	print('Min cost: ' + str(min_difference))
	print("")
	print("")
	return 0

if __name__ == "__main__":
    main()



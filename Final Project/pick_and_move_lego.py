
import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics

USABLE_WIDTH = 10
USABLE_LENGTH = 20

BLOCKSTARTAREA_LENGTH = 10

FINALSTRUCT_LENGTH = USABLE_LENGTH - BLOCKSTARTAREA_LENGTH

LEGO_HEIGHT = 10 #cm

class lego_Mover:
    def __init__(self):
        global LEGO_HEIGHT
        self.lego_height = LEGO_HEIGHT
        self.currentPos = [None, None, None] #length must be 3, [x,y,z]
        self.gripper = intera_interface.Gripper()
        self.limb = intera_interface.Limb('right')
        self.kinematics = sawyer_kinematics('right')
        self.currentLegoPos = [None, None, None]

    def coordToJointPos(self, coord):
        return dict(zip(self.limb._joint_names, self.kinematics.inverse_kinematics(coord)))

    def openGripper(self):
        self.gripper.open()
        rospy.sleep(1)

    def closeGripper(self):
        self.gripper.close()
        rospy.sleep(1)

    def setLegoPosition(self, coords):
        self.currentLegoPos = coords

    def moveArmToTargetPos(self, coords):
        self.limb.set_joint_positions(self.coordToJointPos(coords))
        pass # TODO

    def moveArmToHomePos(self):
        pass # TODO

    def pickLego(self):
        self.openGripper()
        # move down LEGOHEIGHT
        self.closeGripper()
        # move up to a "hover"

    def dropLego(self):
        # move down to LEGOHEIGHT
        self.openGripper()
        # move up to "hover"

# class Blocks:
#     def __init__(self, block_num = 5):
#         self.blocks = []
#         for block in range(0,block_num):
#             temp = Block()
#             self.blocks.append(temp)

class Block:
    def __init__(self):
        global LEGO_HEIGHT
        self.x, self.y = None, None
        self.plane = 0
        self.height = LEGO_HEIGHT

    def setCoords(self, c):
        self.x = c[0]
        self.y = c[1]
        self.plane = c[2]


class target_structure:
    def __init__(self, block_pieces=5):
        global FINALSTRUCT_LENGTH, USABLE_WIDTH
        self.x_range = FINALSTRUCT_LENGTH
        self.y_range = USABLE_WIDTH
        self.num_pieces = block_pieces
        self.structure = [] # will be an x_range by y_range grid with a stacking plane attribute

    def createRandomStructure(self):
        dir = ['N', 'W', 'S', 'E']
        baseBlockNum = int(self.num_pieces/2)
        block = Block()
        x_val = rand(0, self.x_range) # first piece
        y_val = rand(0, self.y_range)
        plane = 0
        coords = [x_val, y_val, plane]
        self.structure.append(coords)
        dir_turn = 0
        for v in range(1, baseBlockNum):
            block = Block()
            # TODO

    def setStructureFromArray(self, structure):
        pass # TODO

    def clearStructure(self):
        self.structure = []





def main():
    global USABLE_LENGTH, USABLE_WIDTH, BLOCKSTARTAREA_LENGTH
    rospy.init_node("lego_mover")
    tarStructure = target_structure()
    tarStructure.createRandomStructure()
    robot = lego_Mover()
    robot.moveArmToHomePos()
    structure = []
    x=0
    y=0
    for i in tarStructure:
        temp = Block()
        temp.setCoords([x,y,0])
        x = x + 1
        if x==BLOCKSTARTAREA_LENGTH:
            x=0
            y=y+1
        if y>USABLE_WIDTH:
            print("NOT ENOUGH AREA TO FIT BLOCKS")
            return -1
        structure.append(temp)


if __name__=="__main__":
    main()



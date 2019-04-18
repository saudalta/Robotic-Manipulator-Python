# This macro shows an example to draw a polygon of radius R and n_sides vertices using the RoboDK API for Python
import numpy as np
from robolink import *    # API to communicate with RoboDK for simulation and offline/online programming
from robodk import *      # Robotics toolbox for industrial robots

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()

# Select a robot (popup is displayed if more than one robot is available)
robot = RDK.Item('KUKA LBR iiwa 14 R820')
base = RDK.Item('KUKA LBR iiwa 14 R820 Base')
panel = RDK.Item('KMP1500', 5)
if not robot.Valid():
    raise Exception('No robot selected or available')

panel_cntrd= Mat()

panel_cntrd[0,0]=0.0
panel_cntrd[0,1]=0.0
panel_cntrd[0,2]=1.0
panel_cntrd[0,3]=750.0

panel_cntrd[1,0]=1.0
panel_cntrd[1,1]=0.0
panel_cntrd[1,2]=0.0
panel_cntrd[1,3]=0.0

panel_cntrd[2,0]=0.0
panel_cntrd[2,1]=1.0
panel_cntrd[2,2]=0.0
panel_cntrd[2,3]=-230.0

panel_cntrd[3,0]=0.0
panel_cntrd[3,1]=0.0
panel_cntrd[3,2]=0.0
panel_cntrd[3,3]=1.0

panel.setPose(panel_cntrd)
#hard coded centered panel position values

def shift_RL(amount):
    if(amount>=0):
        bool =True
    else:
        bool = False
    amount = abs(amount) 
    if(bool == True):
        a=1
    else:
        a=-1
    for i in range(amount):
        v=panel.Pose()
        v[1,3] = v[1,3] + a
        panel.setPose(v)

shift_RL(60)    
#for i in range(4):
#    for j in range(4):
#        print(panel_cntrd[i,j],', ')
#    print('\n')

def rotate_panel(degrees):
    if(degrees>=0):
        bool =True
    else:
        bool = False
    degrees = abs(degrees)    
    if(bool == True):
        ry = roty(pi/180)
    else:
        ry = roty(-pi/180)
    for i in range(degrees):
        v = panel.Pose()
        panel.setPose(v*ry)
        
#rotate_panel(-45)
  
#Robot Program 1
    
home = RDK.Item('Home')
init_pos = RDK.Item('Target 2')
    
# get the current position of the TCP with respect to the reference frame:
# (4x4 matrix representing position and orientation)
target_ref = init_pos.Pose()
pos_ref = target_ref.Pos()
print("Drawing a polygon around the target: ")
print(Pose_2_TxyzRxyz(target_ref))


# move the robot to the first point:
robot.MoveJ(home)
robot.MoveJ(init_pos)

#Move tool to a given point:




# It is important to provide the reference frame and the tool frames when generating programs offline
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
robot.setZoneData(10) # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
robot.setSpeed(50) # Set linear speed in mm/s

# Set the number of sides of the polygon:
n_sides = 6
R = 100



def move_2_point_on_panel(robot, panel,base, x,y):
    robot.MoveJ(init_pos)
    #calculate offsets between end tool and platform
    tool_pos = robot.Pose().Pos()
    panel_pos = panel.Pose().Pos()
    base_pos = base.Pose().Pos()
    
    dxb = base_pos[0]-panel_pos[0]
    dyb = base_pos[1]-panel_pos[1]
    dzb = base_pos[2]-panel_pos[2]
    #if(sqrt(dy**2+dx**2)>=)
    
    dx = tool_pos[0]-panel_pos[0]
    dy = tool_pos[1]-panel_pos[1]
    dz = tool_pos[2]-panel_pos[2]
    
    
    
    target_i = Mat(target_ref)
    pos_i = target_i.Pos()
    #example input to robot: (0,0,0)
    command = Mat(1,3)
    command[0,0] = x
    command[0,1] =y
    command[0,2] = 0
    pos_i[0] = pos_i[0]+ command[0,0]-dx
    pos_i[1] = pos_i[1]+ command[0,1]-dy
    pos_i[2] = pos_i[2]+ command[0,2]
    target_i.setPos(pos_i)
    robot.MoveL(target_i)

move_2_point_on_panel(robot,panel, base, 0,0)
# make a hexagon around reference target:
#for i in range(n_sides+1):
#    ang = i*2*pi/n_sides #angle: 0, 60, 120, ...
#
#    #-----------------------------
#    # Movement relative to the reference frame
#    # Create a copy of the target
#    target_i = Mat(target_ref)
#    pos_i = target_i.Pos()
#    pos_i[0] = pos_i[0] + R*cos(ang)
#    pos_i[1] = pos_i[1] + R*sin(ang)
#    target_i.setPos(pos_i)
#    print("Moving to target %i: angle %.1f" % (i, ang*180/pi))
#    print(str(Pose_2_TxyzRxyz(target_i)))
#    robot.MoveL(target_i)
    
    #-----------------------------
    # Post multiply: relative to the tool
    #target_i = target_ref * rotz(ang) * transl(R,0,0) * rotz(-ang)
    #robot.MoveL(target_i)

# move back to the center, then home:
robot.MoveL(target_ref)

print('Done')
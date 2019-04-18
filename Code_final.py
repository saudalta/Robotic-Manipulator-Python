import numpy as np
import numpy.random as npr
#import matplotlib as plt
#plt.pyplot.scatter(XY[:,1],XY[:,0])
from robolink import *    # API to communicate with RoboDK for simulation and offline/online programming
from robodk import *      # Robotics toolbox for industrial robots
import pandas as pd
# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()


RDK.Render(False) # turn off auto rendering (faster)
# Select a robot (popup is displayed if more than one robot is available)
robot = RDK.Item('KUKA LBR iiwa 14 R820')
base = RDK.Item('KUKA LBR iiwa 14 R820 Base')
panel = RDK.Item('KMP1500', 5)
tool = RDK.Item('ScrewDv1')

if not robot.Valid():
    raise Exception('No robot selected or available')
RDK.Render(True) # Turn rendering ON before starting the simulation
	
# Retrieve the robot reference frame
reference = robot.Parent()

robot.setPoseFrame(reference)


# It is important to provide the reference frame and the tool frames when generating programs offline
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
robot.setZoneData(10) # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
robot.setSpeed(50) # Set linear speed in mm/s

panel_cntrd= Mat()

panel_cntrd[0,0]=0.0
panel_cntrd[0,1]=0.0
panel_cntrd[0,2]=1.0
panel_cntrd[0,3]=750.0 #750

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
        a=5
    else:
        a=-5
    for i in range(0,amount,5):
        v=panel.Pose()
        v[1,3] = v[1,3] + a
        panel.setPose(v)
        
def shift_FB(amount):
    if(amount>=0):
        bool =True
    else:
        bool = False
    amount = abs(amount) 
    if(bool == True):
        a=5
    else:
        a=-5
    for i in range(0,amount,5):
        v=panel.Pose()
        v[0,3] = v[0,3] + a
        panel.setPose(v)

#shift_RL(-40)    
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
txtdata = LoadList("D:/study/ENGR 7401/RobotRDK/final/pts.txt", ',')
dataR = []
for i in range(len(txtdata)):#i starts from 0, and increment by 1 for next line
    #print(txtdata[i])
    dataR.append(txtdata[i])
    
    
home = RDK.Item('Home')
init_pos = RDK.Item('Standby')
    
# get the current position of the TCP with respect to the reference frame:
# (4x4 matrix representing position and orientation)
target_ref = init_pos.Pose()
pos_ref = target_ref.Pos()

# move the robot to the first point:
robot.MoveJ(home)
robot.MoveJ(init_pos)

#Move tool to a given point:

def move_2_point_on_panel(robot, panel, x,y):
    zbuff = 15
    robot.MoveJ(init_pos)
    #calculate offsets between end tool and platform
    tool_pos = robot.Pose().Pos()
    panel_pos = panel.Pose().Pos()
    
    dx = tool_pos[0]-panel_pos[0]
    dy = tool_pos[1]-panel_pos[1]
    dz = tool_pos[2]-panel_pos[2]
    

    target_i = Mat(target_ref)
    pos_i = target_i.Pos()
    print(pos_i)
    #example input to robot: (0,0,0)
    command = Mat(1,3)
    command[0,0] = x
    command[0,1] =y
    command[0,2] =0
    pos_i[0] = pos_i[0]+ command[0,0]-dx
    pos_i[1] = pos_i[1]+ command[0,1]-dy
    pos_i[2] = pos_i[2]+ command[0,2]
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)   #hover above
    
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    #Check error
    
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    robot.MoveJ(init_pos)
    
    if(npr.randint(0,4)==2):
        #print('ERROR on:')
        return 1
    else:
        #print('SUCCESS on:')
        return 0
 
 
def move_2_point_on_panel_top(x,y,z):
    zbuff = 15
    #target_i.setPose(init_pos.Pose())
    robot.MoveJ(init_pos)
    #calculate offsets between end tool and platform
    tool_pos = robot.Pose().Pos()
    panel_pos = panel.Pose().Pos()
    
    dx = tool_pos[0]-panel_pos[0]
    dy = tool_pos[1]-panel_pos[1]
    dz = tool_pos[2]-panel_pos[2]
    

    target_i = Mat(target_ref)
    pos_i = target_i.Pos()
    #print(pos_i)
    #example input to robot: (0,0,0)
    command = Mat(1,3)
    command[0,0] =x
    command[0,1] =y
    command[0,2] =z
    pos_i[0] = pos_i[0]+ command[0,0]-dx
    pos_i[1] = pos_i[1]+ command[0,1]-dy
    pos_i[2] = pos_i[2]+ command[0,2]-dz+zbuff
    
    target_i.setPos(pos_i)
    
    
    # calculate a new approach position 100 mm along the Z axis of the tool with respect to the target
    #approach = target_i.Pose()* transl(0,0,-50)
    #robot.MoveJ(approach)               # linear move to the approach position
  
    
    robot.MoveJ(target_i)   #hover above
    
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    #Check error
    
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    #robot.MoveJ(init_pos)
    
    if(npr.randint(0,4)==2):
        #print('ERROR on:')
        return 1
    else:
        #print('SUCCESS on:')
        return 0   
    
    
    
    

def move_2_point_on_panel_top2(x,y,z):
    zbuff = 20
    #target_i.setPose(init_pos.Pose())
    #robot.MoveJ(init_pos)
    #calculate offsets between end tool and platform
    tool_pos = robot.Pose().Pos()
    panel_pos = panel.Pose().Pos()
    
    dx = tool_pos[0]-panel_pos[0]
    dy = tool_pos[1]-panel_pos[1]
    dz = tool_pos[2]-panel_pos[2]
    
    target_i = robot.Pose()
    pos_i = target_i.Pos()
	
    command = Mat(1,3)
    command[0,0] =x
    command[0,1] =y
    command[0,2] =z
    pos_i[0] = pos_i[0]+ command[0,0]-dx
    pos_i[1] = pos_i[1]+ command[0,1]-dy
    pos_i[2] = pos_i[2]+ command[0,2]-dz+zbuff
    
    target_i.setPos(pos_i)     
    robot.MoveJ(target_i)   #hover above
    
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    #Check error   
    pos_i[2] = pos_i[2] - zbuff #press down
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
    
    pos_i[2] = pos_i[2] + zbuff #return above
    target_i.setPos(pos_i)
    robot.MoveJ(target_i)
           
    if(npr.randint(0,4)==2):
        #print('ERROR on:')
        return 1
    else:
        #print('SUCCESS on:')
        return 0   
    
    
  
#move_2_point_on_panel_top2(-320,-400,470)      
#move_2_point_on_panel_top2(-260,-340,470)     


#define workspace to be 430<Rpoint/base<815
#(based off trial and error tests)
#determine if point in panel coord. frame is accessible
#in base coordinate frame
#e.g. x=..., y = ...
x = -320
y = 0
a = 350 #430
b = 750 #815

#XY = np.array([[-320,-400],[-320,0],[-320,400],[-320,600],[-320,800],[-220,800],[-120,800]])
#XY = np.array([[-320,-400],[-320,0],[-320,400],[-320,600],
             #  [-320,800],[-220,800],[-120,800],[0,800],[120,800],
             #  [220,800],[320,800],[320,600],[320,400],[320,0],[320,-400]])#
XY = np.asarray(dataR)

def check_range(robot, panel, x, y, a, b):

    x_offset = panel.Pose().Pos()[0]
    y_offset = panel.Pose().Pos()[1]

    x_b = x + x_offset
    y_b = y + y_offset

    #print(sqrt(x_b*x_b + y_b*y_b))
    Rb = sqrt(x_b*x_b + y_b*y_b)
#    print('\nx_b = ', x_b,'\ty_b = ', y_b )
#    if(Rb>b or Rb<a):
#        print('\nnot within range')
    return Rb, x_b, y_b

    
def split(XY, robot, panel,a,b):
    n = len(XY)
    Rs = np.zeros((n,1))
    Side1 = np.array([[0,0, 0]])
    Side2 = np.array([[0,0, 0]])
    for i in range(n):
        x = XY[i,0]
        y = XY[i,1]
        Rb, _, _ = check_range(robot, panel, x, y, a, b)
        Rs[i,0]= Rb
        if(x<=0):
            Side1 = np.append(Side1, [[x,y, Rb]], axis = 0)
        else:
            Side2 = np.append(Side2, [[x,y, Rb]], axis = 0)
    Side1 = Side1[1:,:]
    Side2 = Side2[1:,:]       
    #separate data in side 1 side 2, side 1 being negative or zero x (first columng XY[:,0])
    #side 2 positive x     
    return Side1, Side2

s1, s2 = split(XY, robot, panel,a,b)
n1 = s1.shape[0]
n2 = s2.shape[0]
#starting with side 1 (s1)
def attack_side(robot, panel,s1, n1, a, b, sideno):
    errors = 0;
    
    for i in range(n1):
        x1 = s1[i,0]
        y1 = s1[i,1]
        Rb1, _, _ =check_range(robot, panel, x1, y1, a, b)
        while(Rb1 <a):
            if(y1>=0):
                shift_RL(5)
            else:
                shift_RL(-5)
            Rb1, _, _ =check_range(robot, panel, x1, y1, a, b)   
        while(Rb1 > b):
            if(y1>=0):
                shift_RL(-5)
            else:
                shift_RL(5)
            Rb1, _, _ =check_range(robot, panel, x1, y1, a, b)
        c = move_2_point_on_panel_top2(x1, y1,470)
        errors = errors + c
        print("Side number #",sideno," hole number #", i+1)    
        if(c):
            #print("VALIDATION FAILURE ALARM -Press Enter Continue?")            
            input("VALIDATION FAILURE ALARM -Press Enter Continue?")
        print("-----------------------------------------------")
#        while(errors-errors_p):
#            c=wait()
#            if(c=='y'):
#                break;
       
              
    percent_error = errors/n1
    robot.MoveJ(home)
    return percent_error
 
p_e1 = attack_side(robot, panel, s1, n1, a, b,1)
#side 2 (s2)
#first return panel to centered position

while(panel.Pose().Pos()[1]!=0):
    if(panel.Pose().Pos()[1]<0):
        shift_RL(1)
    else:
        if(panel.Pose().Pos()[1]>0):
            shift_RL(-1)
            
#Now, back panel up
while(panel.Pose().Pos()[0]<1550):
    shift_FB(100)
    
 
    
#turn panel 180 degrees
rotate_panel(180)

#turn input points 180 degrees

ry = roty(pi).Rot33()
s2_180 = Mat(3,n2)
for i in range(n2):
    s2_180[0,i] = s2[i,0]
    s2_180[1,i] = s2[i,1]
    s2_180[2,i] =  0

s2_180 = ry*s2_180
for i in range(n2):
    s2_180[2,i] =  0

s2_180= s2_180.tr() #new set of points

#return panel to initial positition X = 750

while(panel.Pose().Pos()[0]>750):
    shift_FB(-100)
# apply holes to other side
  
p_e2 = attack_side(robot, panel, s2_180, n2, a, b,2)

total_error = (p_e1*n1 +p_e2*n2)/(n1+n2)
print('\nTotal Error = ', total_error)
if total_error<=0.25:
    print('task SUCCESS')
else:
    print('task FAILURE')
print('\nDone')

RDK.ShowMessage(" Finish!  ")
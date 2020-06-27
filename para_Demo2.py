import sys
import os

import serial
import time

import keyboard

import pybullet as p
import pybullet_data

import math

if os.name == 'nt':
    from serial.tools.list_ports_windows import comports
elif os.name == 'posix':
    from serial.tools.list_ports_posix import comports
else:
    raise ImportError("Sorry: no implementation for your platform ('{}') available".format(os.name))

"""
PyBullet control functions.
"""

def parse_port_descriptors(portname, name, desc):
    result = {
        'portname' : portname,
        'name' : name
    }
    if desc:
        # split the description by the spaces
        desc = desc.split()
        for d in desc:
            # if a portion of the description has a '=', we use it
            tlist = d.split('=')
            if len(tlist)==2:
                # split by ':' incase its the 'VID:PID=1D50:6015' portion
                keys = [tlist[0]] if ':' not in tlist[0] else tlist[0].split(':')
                vals = [tlist[1]] if ':' not in tlist[1] else tlist[1].split(':')
                for i in range(len(keys)):
                    if i<len(vals):
                        # save the key and value pair to this port's descriptor
                        tkey = keys[i].lower()
                        result[tkey] = vals[i]

    return result

def find_smoothie():

    iterator = sorted(comports())

    for n, (port, name, desc) in enumerate(iterator, 1):
        # parse the USB serial port's description string
        port_data = parse_port_descriptors(port, name, desc)
        # all smoothieboards have a 'name' attribute
        if port_data.get('pid')=='6015':
            return port_data

def create_connection(port_desc):
    if port_desc:
        try:

            return serial.Serial(port_desc.get('portname'), 115200, timeout=0.2)

        except (OSError, serial.SerialException):
            pass


def write_string_to_port(port, data):
    data = (data+'\r\n').encode('UTF-8')
    return port.write(data)

"""
PyBullet control functions.
"""

def move_to_pos(bodyID, position_matrix):
    
    oldJointStates = p.getJointStates(bodyID, [0,1,2])
    
    num_joints = p.getNumJoints(bodyID)
    
    precision = .001
    
    distance = 1000
    iteration = 0
    
    while (distance > precision and iteration < 15):
        new_angles = p.calculateInverseKinematics(bodyID, num_joints - 1, position_matrix, residualThreshold = .0001)
        for joint in range (num_joints - 2):
            p.setJointMotorControl2(bodyID, joint, p.POSITION_CONTROL, new_angles[joint])
        p.stepSimulation()
        #time.sleep(.005)
        new_ee_pos = p.getLinkState(bodyID, num_joints - 1)[4]
        distance = math.sqrt((position_matrix[0] - new_ee_pos[0])**2 + (position_matrix[1] - new_ee_pos[1])**2 + (position_matrix[2] - new_ee_pos[2])**2)

        iteration = iteration + 1
    
    newJointStates = p.getJointStates(bodyID, [0,1,2])
    
    angularDisplacement = [(newJointStates[0][0] - oldJointStates[0][0]), (newJointStates[1][0] - oldJointStates[1][0]), (newJointStates[2][0] - oldJointStates[2][0])]
    print("Angular Displacement: " + str(angularDisplacement))
    jog_angle(angularDisplacement)
    
    #print("Angular Displacement: " + str(angularDisplacement))
    #print("Distance: " + str(distance) + " Number of Iterations: " + str(iteration))
    
def move_dir(bodyID, direction_matrix):
    ee_position = p.getLinkState(bodyID, p.getNumJoints(bodyID) - 1)[4]
    new_ee_position = [ee_position[0] + direction_matrix[0], ee_position[1] + direction_matrix[1], ee_position[2] + direction_matrix[2]]
    move_to_pos(bodyID, new_ee_position)
    p.stepSimulation()

def joint_to_motor(joint_displacements):
    gearRatio = 2.717
    x_angle = -joint_displacements[0]*gearRatio
    y_angle = joint_displacements[1]*gearRatio*gearRatio
    z_angle = -(joint_displacements[2] + joint_displacements[1])*gearRatio*gearRatio
    return [x_angle, y_angle, z_angle]

def jog_x(angle):
    motorangles = joint_to_motor([angle, 0, 0])
    write_string_to_port(this_port, "G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F300")
    print("G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F300")
    
    current_angle = p.getJointState(paraId, 0)[0]
    p.setJointMotorControl2(paraId, 0, p.POSITION_CONTROL, current_angle + angle)
    p.stepSimulation()
    
def jog_y(angle):
    motorangles = joint_to_motor([0, angle, 0])
    write_string_to_port(this_port, "G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F300")
    print("G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F300")
    
    current_angle = p.getJointState(paraId, 1)[0]
    p.setJointMotorControl2(paraId, 1, p.POSITION_CONTROL, current_angle + angle)
    p.stepSimulation()
    
def jog_z(angle):
    motorangles = joint_to_motor([0, 0, angle])
    write_string_to_port(this_port, "G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F2000")
    print("G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F2000")
    
    current_angle = p.getJointState(paraId, 2)[0]
    p.setJointMotorControl2(paraId, 2, p.POSITION_CONTROL, current_angle + angle)
    p.stepSimulation()

def jog_angle(angle_matrix):
    motorangles = joint_to_motor([angle_matrix[0], angle_matrix[1], angle_matrix[2]])
    print("Motor Angles: " + str(motorangles))
    write_string_to_port(this_port, "G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F150")
    print("G0 X" + str(motorangles[0]/6.283*80) + " Y"  + str(motorangles[1]/6.283*80) + " Z"  + str(motorangles[2]/6.283*80) + " F150")
    
"""
Set up PyBullet environment with relevant URDF files.
"""

qqphysicsClient = p.connect(p.GUI)

p.setRealTimeSimulation(1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,0)

planeId = p.loadURDF("plane.urdf")
paraId = p.loadURDF("PARA/para.urdf", [0, 0, 0], p.getQuaternionFromEuler([0,0,0]), useFixedBase = 1)
sphere = p.loadURDF("sphere_small.urdf", [0, .5, .25], p.getQuaternionFromEuler([0,0,0]), useFixedBase = 1)
    
print("Script Started.")

time.sleep(2)

#move_to_pos(paraId, [0, .5, .25])
#p.stepSimulation()
print("Start Pose Executed")
time.sleep(.5)

"""
Begin connection loop.
"""

connected = False

print('not connected')

while True:
    smoothie_desc = find_smoothie()
    if smoothie_desc:
        print('attemping connection to {}...'.format(smoothie_desc['portname']))
        this_port = create_connection(smoothie_desc)
        while this_port and this_port.is_open:
            #Connecting to SmoothieBoard:
            if not connected:
                print('connected')
                write_string_to_port(this_port,'M17')
                write_string_to_port(this_port,'G91')
                print('M17')
                print('G91')
            connected = True
            
            #Different Keystrokes:
            if keyboard.read_key() == "d":
                print("Command: X + .01")
                move_dir(paraId, [.01, 0, 0])
                time.sleep(.001)
        
            if keyboard.read_key() == "a":
                print("Command: X - .01")
                move_dir(paraId, [-.01, 0, 0])
                time.sleep(.001)
        
            if keyboard.read_key() == "e":
                print("Command: Y + .01")
                move_dir(paraId, [0, .01qqqqq, 0])
                time.sleep(.001)
        
            if keyboard.read_key() == "s":
                print("Command: Y - .01")
                move_dir(paraId, [0, -.01, 0])
                time.sleep(.001)

            if keyboard.read_key() == "z":
                print("Command: Z + .01")
                move_dir(paraId, [0, 0, .01])
                time.sleep(.001)

            if keyboard.read_key() == "x":
                print("Command: Z - .01")
                move_dir(paraId, [0, 0, -.01])
                time.sleep(.001)      
            
            #Terminating Program:
            if keyboard.read_key() == "q":
                print("Script Terminated.")
                p.disconnect()
                sys.exit()

            #Addressing Errors:
            try:
                write_string_to_port(this_port,'M999')
                data = this_port.readline().decode('UTF-8')
                if not data=='ok\r\n':
                    print('error reading from smoothie')
                    write_string_to_port(this_port,'M112')
            except (OSError, serial.SerialException):
                break
 
    if connected:
        print('not connected')
    connected = False
    
p.disconnect()
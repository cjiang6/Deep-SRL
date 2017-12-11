# -*- coding: utf-8 -*-
"""
Data collection

Created on Fri Nov  3 19:56:31 2017

@author: cjiang
"""

import time
import os

import numpy as np
import matplotlib.pyplot as plt

import vrep

results_path = "results/"

N_REPETITION = 10
N_EPISODES = 1000
N_STEPS = 10
step_time = 0.5

WAIT_RESPONSE = False  # True: Synchronous response (too much delay)

WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot
STREAMING = vrep.simx_opmode_streaming
BUFFER = vrep.simx_opmode_buffer

if WAIT_RESPONSE:
    MODE_INI = WAIT
    MODE = WAIT
else:
    MODE_INI = STREAMING
    MODE = BUFFER

N_Ultrasonic = 16
ultrasonicID = [-1]*N_Ultrasonic
robotID = -1
pedID = -1
kinect_rgb_ID = -1 
kinect_depth_ID = -1 

#distance=np.full(N_Ultrasonic, -1, dtype=np.float64)
#detect_state=np.full(N_Ultrasonic, -1, dtype=np.float64)

def connect():
    ip = '127.0.0.1'
    port = 19997
    vrep.simxFinish(-1)  # just in case, close all opened connections
    global clientID
    clientID = vrep.simxStart(ip, port, True, True, 3000, 5)
    # Connect to V-REP
    if clientID == -1:
        import sys
        sys.exit('\nV-REP remote API server connection failed (' + ip +
                 ':' + str(port) + '). Is V-REP running?')
    print('Connected to V-REP')
    time.sleep(0.5)
    return

def start():
    stop()
    setup_devices()
    vrep.simxStartSimulation(clientID, ONESHOT)    
    time.sleep(0.5)
#    # Solve a rare bug in the simulator by repeating:
#    setup_devices()
#    vrep.simxStartSimulation(clientID, ONESHOT)
#    time.sleep(0.5)
    return
    
    
def stop():
    vrep.simxStopSimulation(clientID, ONESHOT)
    time.sleep(0.5)
    
def setup_devices():
    global robotID, pedID, kinect_rgb_ID, kinect_depth_ID
    ## setup robot components
    #rc, left_motorID = vrep.simxGetObjectHandle(clientID, 'leftMotor', WAIT)
    #rc, right_motorID = vrep.simxGetObjectHandle(clientID, 'rightMotor', WAIT)
        
    #for idx in range(0, N_Ultrasonic):
    #    item = 'ultrasonicSensor' + str(idx+1)
    #    rc, ultrasonicID[idx] = vrep.simxGetObjectHandle(clientID, item, WAIT)
        
    
    
    #time.sleep(10)
    # send and receive components data
    #vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    #vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
        
    #for i in range(0, N_Ultrasonic):
    #    rc, ds, detected_point, doh, dsn = vrep.simxReadProximitySensor(
    #        clientID, ultrasonicID[i], STREAMING)
    #    distance[i] = detected_point[2]
    #    detect_state[i] = ds
    #print(distance)
    #time.sleep(0.5)
    #for i in range(0, N_Ultrasonic):
    #    rc, ds, detected_point, doh, dsn = vrep.simxReadProximitySensor(
    #        clientID, ultrasonicID[i], STREAMING)
    #    if ds == 1:
    #        distance[i] = detected_point[2]
    #    else:
    #        distance[i] = float('nan')
    #    detect_state[i] = ds
        
    rc, robotID = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx#0', WAIT)
    rc, pedID = vrep.simxGetObjectHandle(clientID, 'Bill', WAIT)
    
    rc, kinect_rgb_ID = vrep.simxGetObjectHandle(clientID, 'kinect_rgb#0', WAIT)
    rc, kinect_depth_ID = vrep.simxGetObjectHandle(clientID, 'kinect_depth#0', WAIT)
                                                   
    rc, resolution, image = vrep.simxGetVisionSensorImage(clientID, kinect_depth_ID, 0, MODE_INI)
    time.sleep(0.5)
    
    robo_init_pos = np.array([-2.5, 2*np.random.rand()-1, 0.13879])    
    rc = vrep.simxSetObjectPosition(clientID, robotID, -1, robo_init_pos, ONESHOT)
    
    ped_init_pos = np.array([2.5, 2*np.random.rand()-1, 0])    
    rc = vrep.simxSetObjectPosition(clientID, pedID, -1, ped_init_pos, ONESHOT)
                                                   
    return

def get_observation(): 
    rc, resolution, depth = vrep.simxGetVisionSensorImage(clientID, kinect_depth_ID, 0, MODE) 
    de = np.array(depth, dtype=np.uint8)
    de.resize([resolution[1], resolution[0], 3])
    plt.imshow(de, origin='lower')
    return de
    
def new_dir(results_path):
    """ Create directory in which results will be saved """
    #global path

    if not os.path.exists(results_path):
        os.makedirs(results_path)
    string_date = time.strftime("%Y_%m_%d_%H_%M", time.gmtime())
    folder = results_path + "/" + string_date
    if not os.path.exists(folder):
        os.makedirs(folder)
    path = folder
    return path
    
    
# main function
path = new_dir(results_path)
connect()
for rep in range(N_REPETITION):
    if not os.path.exists(path + '/' + 'REP' + str(rep+1)):
        os.makedirs(path + '/' + 'REP' + str(rep+1))
    
    for epi in range(N_EPISODES):
        if not os.path.exists(path + '/' + 'REP' + str(rep+1) + '/' + 'EPI' + str(epi+1)):
            os.makedirs(path + '/' + 'REP' + str(rep+1) + '/' + 'EPI' + str(epi+1))
            
        start()
        
        for step in range(N_STEPS):                  
            obs = get_observation()
            #print(obs)
            np.save(path + '/' + 'REP' + str(rep+1) + '/' + 'EPI' + str(epi+1) + '/' + 'step' + str(step+1), obs)
            print('data saved: ' + 'repetition: ' + str(rep+1) + ' ' + 'episode: ' + str(epi+1) + ' ' + 'step: ' + str(step+1))           
            time.sleep(step_time)

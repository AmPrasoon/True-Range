##############################################################
# Date: 01/06/2019                                           #
# Table output: time, Rx, Ry and microphone set with DOP     # 
##############################################################

import numpy as np;
from itertools import combinations
from numpy import *
# import all 

micGrid_x = 2
micGrid_y = 2
micGrid_z = 3.5
signal_range = 4.5
signal_period = 0.2

## Set microphone grid of 100 receivers
## x,y interval = 2m and height z = 3.5m  

def microphone_Grid(robo_pos):
    micGrid = np.zeros((10,10,3))
    micGridList = list()

    for i in range(len((micGrid))):
        for j in range(len(micGrid[0])):
            mic_x = i * micGrid_x
            mic_y = j * micGrid_y
            pos = np.array([mic_x, mic_y, micGrid_z])
            micGrid[(i,j)] = pos

    robo_pos = robo_pos
    for i in range(len((micGrid))):
        for j in range(len(micGrid[0])):
            distance = np.sqrt(np.sum(np.square(robo_pos-micGrid[(i,j)])))
            if distance <= signal_range:
                micGridList.append(micGrid[(i,j)])
    return micGridList

## speaker signal can reach max 4.5m
## Spot 3 microphones with in robot range

def micGrid_inRange(micGridList):
    micGridList = micGridList
    inRangeMic = list(combinations(micGridList,3))
    return inRangeMic

def pythagoras(robo_pos, mic_pos):
    robo_pos = robo_pos
    mic_pos = mic_pos
    distance = np.sqrt(np.sum(np.square(robo_pos-mic_pos)))
    return distance

## Robot navigates on a circular path with center @(9.0,9.0 and radius = 3m)
## Determine its position Rx, Ry (and Rz = 0.5m) wrt time. 

def robo_pos():
    roboLocation = list()
    circleRadius = 3
    theta = 0
    for i in range(0,63):
        theta = i*signal_period
        Rx = 9.0 + circleRadius*cos(theta/2)
        Ry = 9.0 + circleRadius*sin(theta/2)
        Rz = 0.5
        current_roboLoc = np.array([Rx,Ry,Rz,theta])
        DOP(current_roboLoc)

## Dilution of precision to measure errors in above distance measurements
## 

def DOP(robo_pos):
    theta = robo_pos[3]
    robo_pos = robo_pos[0:3]
    micGridList = microphone_Grid(robo_pos)
    best_microphones = micGrid_inRange(micGridList)

    min_PDOP = None
    thebest_chosen_microphones = list()
    for i in range(len(best_microphones)):
        mic_one = best_microphones[i][0]
        mic_two = best_microphones[i][1]
        mic_three = best_microphones[i][2]

        Z_one = pythagoras(robo_pos, mic_one)
        Z_two = pythagoras(robo_pos, mic_two)
        Z_three = pythagoras(robo_pos, mic_three)

        H = np.zeros((3,3))
        H[(0,0)] = -(mic_one[0] - robo_pos[0]) / Z_one
        H[(0,1)] = -(mic_one[1] - robo_pos[1]) / Z_one
        H[(0,2)] = -(mic_one[2] - robo_pos[2]) / Z_one

        H[(1,0)] = -(mic_two[0] - robo_pos[0]) / Z_two
        H[(1,1)] = -(mic_two[1] - robo_pos[1]) / Z_two
        H[(1,2)] = -(mic_two[2] - robo_pos[2]) / Z_two

        H[(2,0)] = -(mic_three[0] - robo_pos[0]) / Z_three
        H[(2,1)] = -(mic_three[1] - robo_pos[1]) / Z_three
        H[(2,2)] = -(mic_three[2] - robo_pos[2]) / Z_three
        H_transpose = np.transpose(H)

        try:
            E = np.linalg.inv(np.dot(H_transpose, H))
        except:
            print(str(best_microphones[i]) + "  ")
        else:
            PDOP = np.sqrt(abs(E[(0,0)] + E[(1,1)] +E[(2,2)]))
            if min_PDOP is None:
                min_PDOP = PDOP
                best_chosen_microphones = best_microphones[i]
            elif min_PDOP > PDOP:
                min_PDOP = PDOP
                best_chosen_microphones = best_microphones[i]
    print("Time:"+str(theta)+" " "Robot Location:" +str(robo_pos)+" ""Best Microphone set-1, set-2 and set-3:"+str(best_chosen_microphones)+" ""PDOP: "+str(min_PDOP))
if __name__ == '__main__':
    robo_pos()
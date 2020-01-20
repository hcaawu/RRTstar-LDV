#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy
#### YOUR IMPORTS GO HERE ####
from rrt import *
#import rrtstar
from rrtstar import *
import matplotlib.pyplot as plt
import csv
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test1.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_wrist_flex_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [-0.15,0.075,-1.008,0,-0.11]

    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)
    lowerlimit,upperlimit=robot.GetActiveDOFLimits();
    print lowerlimit
    print upperlimit
    
    with env:
    	print startconfig
        goalconfig = [0.449,-0.201,-0.151,0,-0.11]
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        rrt=RRTStar(env, robot, startconfig, goalconfig, lowerlimits, upplerlimits)
        path,allcosts,alltimes,samples=rrt.RRTSearch()
        print path
        print allcosts
        '''
        with open('../results/WeightedSampling/s90i30.csv', mode='w') as rrtFile:
            rrtFile_writer = csv.writer(rrtFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            rrtFile_writer.writerow(['Run','samples','Path Cost History'])            
            for i in range(1,21,1):
                rrt=RRTStar(env, robot, startconfig, goalconfig, lowerlimits, upplerlimits)
                path,allcosts,alltimes,samples=rrt.RRTSearch()
                print path
                print allcosts
                row=[str(i),str(samples)]
                for j in range(len(allcosts)):
                    row.append(str(allcosts[j]))
                rrtFile_writer.writerow(row)
        
        
        show_animation=1
        if show_animation:
            rrt.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r', linewidth=5)
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
        # Draw Found Path
        '''
        pathcolor=(1,0,0)
        handles=drawArmPath(env,robot,path,[pathcolor])
        # get trajectory
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())
        for i in range(0,len(path)):
            traj.Insert(i,path[i])  
    replay = 'y'
    while (replay=='y'):  
        planningutils.RetimeActiveDOFTrajectory(traj,robot)
        robot.GetController().SetPath(traj)  
        replay=raw_input("replay? (y/n)\n")  
 
        ### END OF YOUR CODE ###
    waitrobot(robot)
    
    raw_input("Press enter to exit...")


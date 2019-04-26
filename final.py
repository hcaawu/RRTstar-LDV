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
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
    startconfig=[-3.4,-1.4]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)
    '''
    table1=env.GetKinBody('Table1')
    table4=env.GetKinBody('Table4')
    table5=env.GetKinBody('Table5')
    table1.SetTransform([0.70711,0,0,0.70711,-2.3,-1.1,0.74])
    table4.SetTransform([0.70711,0,0,0.70711,-0.578,1.605,0.74])
    table5.SetTransform([1,0,0,0,2.2,0.3,0.74])
    '''
    
    table1=env.GetKinBody('Table1')
    table2=env.GetKinBody('Table2')
    table3=env.GetKinBody('Table3')
    table4=env.GetKinBody('Table4')
    table5=env.GetKinBody('Table5')
    table6=env.GetKinBody('Table6')
    table1.SetTransform([0.70711,0,0,0.70711,0.07657,-1.497,0.74])
    table2.SetTransform([0.70711,0,0,0.70711,1.71,0.198,0.74])
    table3.SetTransform([0.70711,0,0,0.70711,1.71,0.198,0.74])
    table4.SetTransform([1,0,0,0,-0.53473,1.06,0.74])
    table5.SetTransform([1,0,0,0,-2.23025,-0.45607,0.74])
    table6.SetTransform([1,0,0,0,-0.53473,1.06,0.74])
    
    
    with env:
    	robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
    	print startconfig
        #goalconfig = [2.6,-1.3]
        goalconfig = [0,-0.5]
        
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        lowerlimits=[-3.41,-1.41]
        upplerLimits=[0.1,1.41]
        rrt=RRTStar(env, robot, startconfig, goalconfig, lowerlimits, upplerLimits)
        path,allcosts,alltimes,samples=rrt.RRTSearch()
        print path
        print allcosts
        '''
        with open('results/h0s0b80sp001.csv', mode='w') as rrtFile:
            rrtFile_writer = csv.writer(rrtFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            rrtFile_writer.writerow(['Run','samples','Path Cost History'])            
            for i in range(1,3,1):
                rrt=RRTStar(env, robot, startconfig, goalconfig, [-3.41, 3.41], [-1.41, 1.41])
                path,allcosts,alltimes,samples=rrt.RRTSearch()
                print path
                print allcosts
                row=[str(i),str(samples)]
                for j in range(len(allcosts)):
                    row.append(str(allcosts[j]))
                rrtFile_writer.writerow(row)
        '''
        
        show_animation=1
        if show_animation:
            rrt.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r', linewidth=5)
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
        # Draw Found Path
        
        pathcolor=(1,0,0)
        handles=drawArmPath(env,robot,path,[pathcolor])
        # get trajectory
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())
        for i in range(0,len(path)):
            traj.Insert(i,path[i][0:2])    
    replay = 'y'
    while (replay=='y'):
        planningutils.RetimeActiveDOFTrajectory(traj,robot)
        robot.GetController().SetPath(traj)
        replay=raw_input("replay? (y/n)\n")         
        ### END OF YOUR CODE ###
    waitrobot(robot)
    
    raw_input("Press enter to exit...")


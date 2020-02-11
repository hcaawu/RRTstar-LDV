from numpy import *
import matplotlib.pyplot as plt
# RRT Search
def rrtSearch(RRT,start,goal,bias=0.06,stepsize=0.28,rrtType=1,K=1):
	setting=[bias,stepsize,rrtType,K]
	command='Search '+str(start)+str(goal)+str(setting)
	#print command
	out=RRT.SendCommand(command)
	out=out.splitlines()
	timepassed=float(out[0])
	samples=int(out[1])
	pathout=out[2:len(out)]
	path=strPathToList(pathout)
	return path,timepassed,samples

def smoothPath(RRT,smooth=200):
	command='Smooth '+str(smooth)
	out=RRT.SendCommand(command)
	out=out.splitlines()
	pathLen=out[0]
	pathSmoothed=out[2:len(out)]
	timeSmooth=float(out[1])
	pathL=[]
	for p in pathLen.split(","):
		pathL.append(int(p))
	pathSmoothed=strPathToList(pathSmoothed)
	print pathSmoothed
	return pathL,pathSmoothed,timeSmooth

# Convert output string path to list path
def strPathToList(strPath):
	listPath=[]
	for config in strPath:
		config=config.split(",")
		qj=[]
		for j in range(len(config)-1):
			qj.append(float(config[j]))
		listPath.append(qj)
	return listPath

# draw end effector
def drawArmPath(env,robot,path,color):
	drawpoint=[]
	handles=[]
	
	robot.SetActiveManipulator('leftarm')
        leftarm = robot.GetActiveManipulator();
	for config in path:
		robot.SetActiveDOFValues(config);
		trans=leftarm.GetEndEffectorTransform()
		drawpoint.append(trans[0:3,3])
	color=array(color*len(array(drawpoint)))	
	handles.append(env.drawlinestrip(points=array(drawpoint),linewidth=3,colors=color))
	
	'''
	path3=[]
	for q in path:
		q.append(0.3)
		path3.append(array(q))
	
	color=array(color*len(array(path3)))
	handles.append(env.drawlinestrip(points=array(path3),linewidth=3,colors=color))
	return handles
	'''
# Time vs Bias plot
def plotTimeVSBias(Time,Bias,sigmaplus,sigmaminus):
    plt.plot(Bias,Time)
    plt.plot(Bias,sigmaplus,color='r')
    plt.plot(Bias,sigmaminus,color='r')
    plt.xlabel("Goal bias")
    plt.ylabel("Average time spent (s)")
    plt.title("Average time vs. goal bias")
    plt.legend(['average time','+sigma','-sigma'])
    plt.show()

def plotPathLength(pathL):
	plt.plot(range(1,len(pathL)+1),pathL)
	plt.xlabel("Smooth times")
	plt.ylabel("Path length")
	plt.title("Path length vs. smooth times")
	plt.show()
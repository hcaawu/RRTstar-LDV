import openravepy
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time

class Node():
    def __init__(self,q):
        self.q=q
        self.cost=0.0
        self.parent=None
        self.visibility = float('inf')
class RRTStar():
    def __init__(self, env, robot, start, goal, xlimits, ylimits, goalBias=10 ,steersize=0.3):
        self.env=env
        self.robot=robot
        self.start=Node(start)
        self.goal=Node(goal)
        self.xlowerlimit=xlimits[0]
        self.xupperlimit=xlimits[1]
        self.ylowerlimit=ylimits[0]
        self.yupperlimit=ylimits[1]
        self.goalBias=goalBias
        self.steersize=steersize
        self.checksize=0.2
        self.failSparsity=0.1
        self.newHeuristic= 0
        self.samplingStrategyBias=50
        self.maxIter=3000
        self.r=self.steersize
        self.timestart = 0.0
        self.timefs = 0.0
        self.timeend =0.0
        #self.totalcost = 0.0

    def RRTSearch(self, animation=1):
        timestart = time.time()
        #random.seed(0)
        firstFound = False
        self.nodeTree=[self.start]
        self.failNodes=[]
        for i in range(self.maxIter):
            #print len(self.failNodes)
            if firstFound:
                if self.failNodes and random.randint(0, 100) > self.samplingStrategyBias:
                    rndQ = self.get_point_around_failnodes()
                    #rndQ = self.get_random_point()
                else:  # rnegular sampling strategy
                    rndQ = self.get_random_point()
            else:
                rndQ = self.get_random_point()         
            minidx = self.GetNearestListIndex(self.nodeTree, rndQ)
            newNode = self.steer(rndQ, minidx)

            if self.__CollisionCheck(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                #self.robot.SetActiveDOFValues(newNode.q);
                #self.env.UpdatePublishedBodies();
                #print newNode.q
                if newNode.parent == None:
                    pass
                else:
                    self.nodeTree.append(newNode)
                    #if not firstFound:
                    self.update_failNodes(newNode)
                    self.rewire(newNode, nearinds, minidx)
            if animation and i % 5 == 0:
                self.DrawGraph(rndQ)

            if not firstFound and i % 10 == 0:
                #lastIndex =self.get_best_last_index()
                bestpath, minpathcost = self.get_best_last_index()
                if bestpath is None:
                #if lastIndex is None:
                    firstFound = False
                else:
                    firstFound = True
                    allcosts = []
                    allcosts.append(minpathcost)
                    firstIter = i
                    timefs = time.time() - timestart
                    #path = self.gen_final_course(lastIndex)
                    #self.cal_totalcost(path)
                    print "First Found! Iter: "+ str(i)+". Cost: "+ str(minpathcost) + ". Time: " + str(timefs)

            if firstFound and i % 50 == 0:
                #lastIndex =self.get_best_last_index()
                bestpath, minpathcost = self.get_best_last_index()
                allcosts.append(minpathcost)
                #path = self.gen_final_course(lastIndex)
                #self.cal_totalcost(path)
                print "Iter: "+str(i)+". Cost: "+str(minpathcost)
                

        # generate coruse
        #lastIndex = self.get_best_last_index()
        #if lastIndex is None:
        #    return None
        bestpath, minpathcost = self.get_best_last_index()
        allcosts.append(minpathcost)
        #path = self.gen_final_course(lastIndex)
        timeend = time.time() - timestart
        print "Time: " + str(timeend)
        #self.cal_totalcost(path)
        #print self.totalcost
        return bestpath, allcosts, len(self.nodeTree), timefs, firstIter

    def update_failNodes(self, newNode):
        if newNode.parent == None:
            pass
        else:
            dx = newNode.q[0] - self.nodeTree[newNode.parent].q[0]
            dy = newNode.q[1] - self.nodeTree[newNode.parent].q[1]
            theta = math.atan2(dy, dx)
            step = 0
            tmpNode = copy.deepcopy(newNode)
            while self.__CollisionCheck(tmpNode):
                tmpNode.q[0] += self.failSparsity * math.cos(theta)
                tmpNode.q[1] += self.failSparsity * math.sin(theta)
                step += 1
            step -= 1
            failNodeQ = [tmpNode.q[0], tmpNode.q[1]]
            failNodeQ[0] = newNode.q[0] + step * self.failSparsity * math.cos(theta)
            failNodeQ[1] = newNode.q[1] + step * self.failSparsity * math.sin(theta)
            
            if not self.failNodes:
                self.failNodes.append(failNodeQ)
            else:
                mindist = self.GetNearestNeighborDist(self.failNodes,failNodeQ)
                if  mindist>self.failSparsity:
                    self.failNodes.append(failNodeQ)
        


    def choose_parent(self, newNode, nearinds):
        if not nearinds:
            return newNode

        costlist =[]
        vislist= []
        dlist = []
        for i in nearinds:
            dx = newNode.q[0] - self.nodeTree[i].q[0]
            dy = newNode.q[1] - self.nodeTree[i].q[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeTree[i], theta, d):
                tmpvis =  self.cal_visibility(self.nodeTree[i], theta, d)
                self.nodeTree[i].cost=self.cal_cost2come(i)
                costlist.append(self.nodeTree[i].cost + d)
                vislist.append(tmpvis)
                if self.newHeuristic:
                    dlist.append(self.nodeTree[i].cost + d +tmpvis)
                else:
                    dlist.append(self.nodeTree[i].cost + d)
            else:
                costlist.append(float("inf"))
                vislist.append(float("inf"))
                dlist.append(float("inf"))

        mind = min(dlist)
        minind = nearinds[dlist.index(mind)]

        if mind == float("inf"):
            #print("mind is inf")
            newNode.parent = None
            return newNode

        newNode.cost = costlist[dlist.index(mind)]
        newNode.visibility = vislist[dlist.index(mind)]
        #print "cost: "+ str(newNode.cost) + "  visibility : " + str(newNode.visibility)
        newNode.parent = minind

        return newNode

    def steer(self, rndQ, minidx):

        # expand tree
        nearestNode = self.nodeTree[minidx]
        theta = math.atan2(rndQ[1] - nearestNode.q[1], rndQ[0] - nearestNode.q[0])
        newNode = Node(rndQ)
        currentDistance = math.sqrt(
            (rndQ[1] - nearestNode.q[1]) ** 2 + (rndQ[0] - nearestNode.q[0]) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.steersize:
            newNode.cost = nearestNode.cost + currentDistance
        else:
            newNode.q[0] = nearestNode.q[0] + self.steersize * math.cos(theta)
            newNode.q[1] = nearestNode.q[1] + self.steersize * math.sin(theta)
        newNode.cost = nearestNode.cost + self.steersize
        newNode.parent = minidx
        return newNode

    def get_point_around_failnodes(self):
        a = random.randint(0, len(self.failNodes)-1)
        failrndC = self.failNodes[a]
        randsize=self.steersize*self.failSparsity*10
        rndQ = [failrndC[0]+random.uniform(-randsize, randsize),
               failrndC[1]+random.uniform(-randsize, randsize)]
        while not self.__CollisionCheckQ(rndQ):
            rndQ = [failrndC[0]+random.uniform(-randsize, randsize),
                    failrndC[1]+random.uniform(-randsize, randsize)]

        return rndQ

    def get_random_point(self):

        if random.randint(0, 100) > self.goalBias:
            rndQ = [random.uniform(self.xlowerlimit, self.xupperlimit),
                   random.uniform(self.ylowerlimit, self.yupperlimit)]
        else:  # goal point sampling
            rndQ = [self.goal.q[0], self.goal.q[1]]

        return rndQ

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.q[0], node.q[1]) for node in self.nodeTree]
        goalinds = [disglist.index(i) for i in disglist if i <= 0]

        if not goalinds:
            return None, float("inf")

        pathcost = []
        path = []
        for j in range(len(goalinds)):
            path.append(self.gen_final_course(goalinds[j]))
            pathcost.append(self.cal_totalcost(path[j]))

        mincost = min(pathcost)
        for j in range(len(goalinds)):
            if pathcost[j] == mincost:
                return path[j], mincost

        #mincost = min([self.nodeTree[i].cost for i in goalinds])
        #for i in goalinds:
        #    if self.nodeTree[i].cost == mincost:
        #        return i

        return None, float("inf")

    def gen_final_course(self, goalind):
        path = [[self.goal.q[0], self.goal.q[1]]]
        while self.nodeTree[goalind].parent is not None:
            node = self.nodeTree[goalind]
            path.append([node.q[0], node.q[1]])
            goalind = node.parent
        path.append([self.start.q[0], self.start.q[1]])
        path.reverse()
        return path

    def cal_cost2come(self, ind):
        path=[]
        while self.nodeTree[ind].parent is not None:
            node = self.nodeTree[ind]
            path.append([node.q[0], node.q[1]])
            ind = node.parent
        path.reverse()
        cost2come=self.cal_totalcost(path)
        return cost2come

    def cal_totalcost(self, path):
        totalcost = 0
        for i in range(len(path)):
            if i!= (len(path)-1):
                totalcost += math.sqrt ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)
        return totalcost

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.goal.q[0], y - self.goal.q[1]])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeTree)
        self.r = min(50.0 * math.sqrt((math.log(nnode) / nnode)), self.steersize)
        #  r = self.expandDis * 5.0
        dlist = [(node.q[0] - newNode.q[0]) ** 2 +
                 (node.q[1] - newNode.q[1]) ** 2 for node in self.nodeTree]
        nearinds = [dlist.index(i) for i in dlist if i <= self.r ]
        return nearinds

    def rewire(self, newNode, nearinds, minind):
        nnode = len(self.nodeTree)
        for i in nearinds:
            if i!=minind:
                nearNode = self.nodeTree[i]

                dx = newNode.q[0] - nearNode.q[0]
                dy = newNode.q[1] - nearNode.q[1]
                d = math.sqrt(dx ** 2 + dy ** 2)

                scost = newNode.cost + d

                if nearNode.cost > scost:
                    theta = math.atan2(dy, dx)
                    if self.check_collision_extend(nearNode, theta, d):
                        nearNode.parent = nnode - 1
                        nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)
        '''
        for i in range(int(d / self.steersize)):
            tmpNode.q[0] += self.steersize * math.cos(theta)
            tmpNode.q[1] += self.steersize * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False
        return True
        '''
        for i in range(int(d / self.checksize)):
            tmpNode.q[0] += self.checksize * math.cos(theta)
            tmpNode.q[1] += self.checksize * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False
        return True

    def cal_visibility(self, nearNode, theta, d):
        nodevis = 0.0
        step = 0
        tmpNode = copy.deepcopy(nearNode)
        checksize=0.2
        while self.__CollisionCheck(tmpNode):
            tmpNode.q[0] += checksize * math.cos(theta)
            tmpNode.q[1] += checksize * math.sin(theta)
            step += 1

        nodevis = 10/(step*checksize -d)

        return nodevis

    def GetNearestListIndex(self, nodeTree, rndQ):
        dlist = [(node.q[0] - rndQ[0]) ** 2 + (node.q[1] - rndQ[1])** 2 for node in nodeTree]
        minidx = dlist.index(min(dlist))
        return minidx

    def GetNearestNeighborDist(self, failNodes, rndQ):
        dlist = [(node[0] - rndQ[0]) ** 2 + (node[1] - rndQ[1])** 2 for node in failNodes]
        mindist = min(dlist)
        return mindist

    def __CollisionCheck(self, node):
        self.robot.SetActiveDOFValues(node.q);
        #self.env.UpdatePublishedBodies();
        if self.env.CheckCollision(self.robot):
            return False
        else:
            return True
    def __CollisionCheckQ(self, Q):
        self.robot.SetActiveDOFValues(Q);
        #self.env.UpdatePublishedBodies();
        if self.env.CheckCollision(self.robot):
            return False
        else:
            return True


    def DrawGraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeTree:
            if node.parent is not None:
                plt.plot([node.q[0], self.nodeTree[node.parent].q[0]], [
                         node.q[1], self.nodeTree[node.parent].q[1]], "-b")
        for q in self.failNodes:
            plt.plot(q[0],q[1],'xr')
        plt.plot(self.start.q[0], self.start.q[1], "oy")
        plt.plot(self.goal.q[0], self.goal.q[1], "oy")
        plt.axis([-3.5, 3.5, -1.5, 1.5])
        plt.grid(True)
        plt.pause(0.01)
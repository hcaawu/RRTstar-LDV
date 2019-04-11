import openravepy
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

class Node():
    def __init__(self,q):
        self.q=q
        self.cost=0.0
        self.parent=None

class RRTStar():
    def __init__(self, env, robot, start, goal, xlimits, ylimits, goalBias=10 ,steersize=0.8):
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
        self.maxIter=800

    def RRTSearch(self, animation=1):
        random.seed(0)
        self.nodeTree=[self.start]
        for i in range(self.maxIter):
            rndQ = self.get_random_point()           
            minidx = self.GetNearestListIndex(self.nodeTree, rndQ)
            newNode = self.steer(rndQ, minidx)

            if self.__CollisionCheck(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.robot.SetActiveDOFValues(newNode.q);
                self.env.UpdatePublishedBodies();
                self.nodeTree.append(newNode)
                self.rewire(newNode, nearinds, minidx)
            if animation and i % 5 == 0:
                self.DrawGraph(rndQ)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if not nearinds:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.q[0] - self.nodeTree[i].q[0]
            dy = newNode.q[1] - self.nodeTree[i].q[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeTree[i], theta, d):
                dlist.append(self.nodeTree[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
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
            pass
        else:
            newNode.q[0] = nearestNode.q[0] + self.steersize * math.cos(theta)
            newNode.q[1] = nearestNode.q[1] + self.steersize * math.sin(theta)
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalBias:
            rndQ = [random.uniform(self.xlowerlimit, self.xupperlimit),
                   random.uniform(self.ylowerlimit, self.yupperlimit)]
        else:  # goal point sampling
            rndQ = [2.6, -1.3]

        return rndQ

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.q[0], node.q[1]) for node in self.nodeTree]
        goalinds = [disglist.index(i) for i in disglist if i <= self.steersize]

        if not goalinds:
            return None

        mincost = min([self.nodeTree[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeTree[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.goal.q[0], self.goal.q[1]]]
        while self.nodeTree[goalind].parent is not None:
            node = self.nodeTree[goalind]
            path.append([node.q[0], node.q[1]])
            goalind = node.parent
        path.append([self.start.q[0], self.start.q[1]])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.goal.q[0], y - self.goal.q[1]])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeTree)
        r = min(20.0 * math.sqrt((math.log(nnode) / nnode)),self.steersize)
        #  r = self.expandDis * 5.0
        dlist = [(node.q[0] - newNode.q[0]) ** 2 +
                 (node.q[1] - newNode.q[1]) ** 2 for node in self.nodeTree]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
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
        checksize=0.3
        '''
        for i in range(int(d / self.steersize)):
            tmpNode.q[0] += self.steersize * math.cos(theta)
            tmpNode.q[1] += self.steersize * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False
        return True
        '''
        for i in range(int(d / checksize)):
            tmpNode.q[0] += checksize * math.cos(theta)
            tmpNode.q[1] += checksize * math.sin(theta)
            if not self.__CollisionCheck(tmpNode):
                return False
        return True

    def GetNearestListIndex(self, nodeTree, rndQ):
        dlist = [(node.q[0] - rndQ[0]) ** 2 + (node.q[1] - rndQ[1])** 2 for node in nodeTree]
        minidx = dlist.index(min(dlist))

        return minidx

    def __CollisionCheck(self, node):
        self.robot.SetActiveDOFValues(node.q);
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
                         node.q[1], self.nodeTree[node.parent].q[1]], "-g")

        plt.plot(self.start.q[0], self.start.q[1], "xr")
        plt.plot(self.goal.q[0], self.goal.q[1], "xr")
        plt.axis([-3.5, 3.5, -1.5, 1.5])
        plt.grid(True)
        plt.pause(0.01)
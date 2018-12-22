from square import square
from board import board
import math
import sys
import cv2
import numpy as np
class search(object):

    closedset = set()
    openset = set()
    came_from = {} # the next step back up the path from [node]
    g_score = {} # current path cost from the start node to [node]
    f_score = {} # estimated path cost from [node] to the goal node
    theboard = ""

    def __init__(self,img,length,height,startx,starty,goalx,goaly):

        i = 0
        kernel = np.ones((10,10), np.uint8)
        img = cv2.dilate(img, kernel, iterations=1)
        #img2 = np.zeros((img.shape))
        return_path = self.run(img, length, height,startx,starty,goalx,goaly)
        for sq in return_path:
            self.theboard.squares[sq.x][sq.y].state = '*'
            img[sq.y][sq.x] = 255
            i = i + 1
        #print self.theboard
        cv2.imwrite("./output.png",img)

    def run(self,img,length,height,startx,starty,goalx,goaly):
        """ Run the search """

        # setup board
        self.theboard = board(img,length,height,startx,starty,goalx,goaly)
        #print self.theboard
        start = self.theboard.start
        print("start:",start)
        goal = self.theboard.goal
        print("goal:", goal)
        # initialise start node
        self.g_score[start] = 0
        self.f_score[start] = self.g_score[start] + self.heuristic_cost_estimate(start,goal)
        self.openset.add(start)

        while self.count(self.openset) > 0:
            # while we still have nodes left to evaluate

            # pick the next node to evaluate as the one we estimate has the shortest path cost to reach the goal node
            # that hasn't already been evaluated
            f_score_sorted = sorted(self.f_score, key=lambda square: self.g_score[square] + self.heuristic_cost_estimate(square,goal))
            i = 0
            for i in range(len(f_score_sorted)-1):
                if(f_score_sorted[i] not in self.closedset):
                    break

            current = f_score_sorted[i]

            if current == goal:
                return self.reconstruct_path(goal)

            try:
                self.openset.remove(current)
            except KeyError,e:
                pass

            self.closedset.add(current)
            for neighbour in self.neighbour_nodes(current):
                if neighbour not in self.closedset:
                    if abs(neighbour.x - current.x) == 1 and abs(neighbour.y - current.y) == 1:
                        temp_g_score = self.g_score[current] + 1.5
                    else:
                        temp_g_score = self.g_score[current] + 1
                    if (neighbour not in self.openset) or (temp_g_score < self.g_score[neighbour]): 
                        # if the neighbour node has not yet been evaluated yet, then we evaluate it
                        # or, if we have just found a shorter way to reach neighbour from the start node, 
                        # then we replace the previous route to get to neighbour, with this new quicker route
                        self.came_from[neighbour] = current
                        self.g_score[neighbour] = temp_g_score
                        self.f_score[neighbour] = self.g_score[neighbour] + self.heuristic_cost_estimate(neighbour,goal)
            
                        if neighbour not in self.openset:
                            self.openset.add(neighbour)
        
      
        print "Reached the end of nodes to expand, failure"       


    def neighbour_nodes(self,node):
        """ Generate a set of neighbouring nodes """
        neighbours = set()
        if node.north != 0:
            neighbours.add(node.north)
        if node.east != 0:
            neighbours.add(node.east)
        if node.west != 0:
            neighbours.add(node.west)
        if node.south != 0:
            neighbours.add(node.south)

        '''if node.northwest != 0:
            neighbours.add(node.northwest)
        if node.northeast != 0:
            neighbours.add(node.northeast)
        if node.southwest != 0:
            neighbours.add(node.southwest)
        if node.southeast != 0:
            neighbours.add(node.southeast)'''
        
        return neighbours


    def distance_to(self,start_node,end_node):
        """ The distance in a straight line between two points on the board """
        x = 3*(start_node.x - end_node.x)
        y = start_node.y - end_node.y

        return abs(x) + abs(y)

    def evaluation_function(self,node,goal):
        """ Our evaluation function is the distance_to function plus the cost of the path so far """
        return (node.self.distance_to(goal) + node.path_cost)

    def heuristic_cost_estimate(self,start_node,end_node):
        heuristic = self.distance_to(start_node,end_node)
        return heuristic 

    def reconstruct_path(self, current_node):
        """ Reconstruct the path recursively by traversing back through the came_from list """

        try: 
            self.came_from[current_node]
            p = self.reconstruct_path(self.came_from[current_node])
            return_path = []
            return_path.extend(p)
            return_path.append(current_node)
            return return_path
        except KeyError,e:
            # we have reached the start node
            return [current_node]

    def count(self,set_to_count):
        total_count = 0
        for i in set_to_count:
            total_count = total_count + 1
        return total_count
'''img = cv2.imread("./map.png",0)
length = img.shape[1]
height = img.shape[0]
startx = 170
starty = 320
goalx =  250
goaly = 285
search(img,length,height,startx,starty,goalx,goaly)'''

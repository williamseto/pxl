#! /usr/bin/env python

import numpy as np
from optparse import OptionParser
import matplotlib.pyplot as plt
from heapq import *
from functools import partial

import cv2

def manhattan(s1, s2):
    return abs(s1[0]-s2[0]) + abs(s1[1]-s2[1])

class Searcher():
    def __init__(self, filelines):

        self.dim = int(filelines[1])
        self.start = map(int, filelines[3].split(","))
        i = 5
        self.target_traj = []
        self.grid = np.zeros([self.dim, self.dim])
        while filelines[i] != "B":
            self.target_traj.append(map(int,lines[i].split(",")))
            i = i + 1
        i = i + 1
        for k in range(self.dim):
            self.grid[k,:] = map(int, lines[i+k].split(","))

        #prune feasible goals from the trajectory
        self.goals = np.empty((0,3))
        for time, pos in enumerate(self.target_traj):
            if manhattan(self.start, pos) <= time:
                self.goals = np.vstack((self.goals, pos+[time]))
                #print (pos+[time])

        # print self.goals
        # exit(1)

        # plt.matshow(self.grid)
        # plt.colorbar(orientation='vertical')
        # plt.show()

    def GetStartState(self):

        # state is ((robot pos, time), path, cumulative g, h)
        start = self.start+[0]
        start_state = [start, [], 0, self.Heuristic(start)]
        return start_state

    def Heuristic(self, state):

        # min_dist = 99999999
        # for goal in self.goals:

        #     #if it's not possible for the state to reach this goal
        #     if state[2] >= goal[2]:
        #         continue
        #     dist = manhattan(state[:2], goal[:2]) + (goal[2]-state[2])
        #     if dist < min_dist:
        #         min_dist = dist

        #get all goals that we can still reach (timewise)
        valid_goals = self.goals[(self.goals[:,2] >= state[2]), :]
        #print valid_goals
        dists = map(lambda x : manhattan(x, state), valid_goals)
        #print dists
        #print np.maximum(valid_goals[:,2], dists)
        return min(np.maximum(valid_goals[:,2], dists))

    def GetSuccessors(self, state):

        succs = []
        pos = state[0]

        # if pos[2] >= self.goals[-1,-1]:
        #     #print ("returning", self.goals[-1,:], pos[2], self.goals[-1,-1])
        #     return succs
        # #left
        if pos[1] > 0:

            succs.append([[pos[0], pos[1]-1, pos[2]+1]])
        #     new_state = [pos[0], pos[1]-1, pos[2]+1]
        #     succs.append([[pos[0], pos[1]-1, pos[2]+1], \
        #                   state[1]+ [new_state[:2]], \
        #                   state[2]+ self.grid[new_state[0], new_state[1]], \
        #                   self.Heuristic(new_state)])

        # #right
        if pos[1] < self.dim-1:
            succs.append([[pos[0], pos[1]+1, pos[2]+1]])
        #     new_state = [pos[0], pos[1]+1, pos[2]+1]
        #     succs.append([new_state, \
        #                   state[1]+ [new_state[:2]], \
        #                   state[2]+ self.grid[new_state[0], new_state[1]], \
        #                   self.Heuristic(new_state)])

        # #up
        if pos[0] > 0:
            succs.append([[pos[0]-1, pos[1], pos[2]+1]])
        #     new_state = [pos[0]-1, pos[1], pos[2]+1]
        #     succs.append([new_state, \
        #                   state[1]+ [new_state[:2]], \
        #                   state[2]+ self.grid[new_state[0], new_state[1]], \
        #                   self.Heuristic(new_state)])

        # #down   
        if pos[0] < self.dim-1:
            succs.append([[pos[0]+1, pos[1], pos[2]+1]])
        #     new_state = [pos[0]+1, pos[1], pos[2]+1]
        #     succs.append([new_state, \
        #                   state[1]+ [new_state[:2]], \
        #                   state[2]+ self.grid[new_state[0], new_state[1]], \
        #                   self.Heuristic(new_state)])

        # #do nothing
        # new_state = [pos[0], pos[1], pos[2]+1]
        # succs.append([new_state, \
        #           state[1]+ [new_state[:2]], \
        #           state[2]+ self.grid[new_state[0], new_state[1]], \
        #           self.Heuristic(new_state)])  

        return succs

    def in_bounds(self, state):
        (x, y) = state
        return 0 <= x < self.dim and 0 <= y < self.dim

    def GetSuccessors2(self, state):
        (x, y) = state
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]

        results = filter(self.in_bounds, results)
        return results

    def IsGoalState(self, state):
        return state[:2] == self.target_traj[state[2]]

def reconstruct_path(camefrom, state):

    current = tuple(state[:2])
    total_path = [state]

    while current in camefrom.keys():
        current = camefrom[current]
        total_path.append(current)
    return total_path

if __name__ == '__main__':

    usage = "usage: %prog filename"
    parser = OptionParser(usage=usage)
    (options, args) = parser.parse_args()

    if not args:
        parser.print_help()
        exit(0)


    with open(args[0]) as gridfile:    
        lines = gridfile.read().splitlines()

    searcher = Searcher(lines)

    goal = tuple(searcher.target_traj[0])


    closed = set()
    gscore = {}
    camefrom = {}

    fringe = []
    start_state = tuple(searcher.start)

    heappush(fringe, (0, start_state))

    gscore[start_state] = 0


    im = np.array(searcher.grid)
    im = ((im / np.max(im)))
    im = im.astype(np.float32)
    im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR, im)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)

    nodes_expanded = 0

    while len(fringe) > 0:

        el = heappop(fringe)


        nodes_expanded = nodes_expanded + 1

        curr_state = el[1]

        if curr_state == goal:

            print "SUCCESS"
            print reconstruct_path(camefrom, curr_state)
            exit(0)


        # im[curr_state] = 255
        # cv2.imshow('image',im)
        # cv2.waitKey(1)
        # print nodes_expanded


        # if curr_state not in closed:
        #     closed.add(curr_state)

        successors = searcher.GetSuccessors2(curr_state)

        for state in successors:

            # The distance from start to a successor
            tentative_gScore = gscore[curr_state] + searcher.grid[state]

            if state not in gscore or tentative_gScore < gscore[state]:
                # path best until now, record it
                gscore[state] = tentative_gScore
                priority = tentative_gScore + manhattan(goal, state)
                heappush(fringe, (priority, state))
                camefrom[state] = curr_state



        #s = raw_input("press enter to continue")
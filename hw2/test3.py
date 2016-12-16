#! /usr/bin/env python


from implementation import *
import numpy as np
from optparse import OptionParser
import matplotlib.pyplot as plt
from heapq import *
from functools import partial
import pickle
import time
import cv2
import matplotlib.pyplot as plt
from scipy import ndimage

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
        self.goals = np.empty((0,3), dtype=np.int)
        self.goals_time_set = set()
        self.goals_set = set()
        for t, pos in enumerate(self.target_traj):
            if manhattan(self.start, pos) <= t:
                self.goals = np.vstack((self.goals, pos+[t]))
                self.goals_time_set.add(tuple(pos) + (t,))
                self.goals_set.add(tuple(pos))


        # self.heuristicgrid = np.zeros([self.dim, self.dim], dtype=np.int)
        # #precompute heuristic for all locations on grid

        # # do distance transform
        t1 = time.time()
        # valid_goals = list(self.goals_set)
        # print valid_goals
        # for i in range(self.dim):
        #     for j in range(self.dim):
        #         dists = map(lambda x : manhattan(x, (i,j)), valid_goals)
        #         print dists
        #         print len(valid_goals)
        #         exit(0)
        #         self.heuristicgrid[i,j] = min(dists)
        #         #self.heuristicgrid[i,j] = min(np.maximum(self.goals[:,2], dists))

        valid_goals = list(self.goals_set)
        self.heuristicgrid = np.ones([self.dim, self.dim], dtype=np.int)
        for (x,y) in valid_goals:
            self.heuristicgrid[x,y] = 0

        self.heuristicgrid = ndimage.distance_transform_edt(self.heuristicgrid)
        print("time to compute heuristic grid", time.time() - t1)
        #exit(1)
        # print self.goals
        # print self.goals[:,:2] != (9,2)
        # print np.any(self.goals[:,:2] != (9,2), axis=1)
        # print self.goals[np.all(self.goals[:,:2] == (2,2), axis=1),:]
        # exit(1)

def reconstruct_path(came_from, start, goal, costmap):
    current = goal
    path = [current]

    tmp = costmap[current]
    total_cost = tmp
    min_cost_in_path = tmp
    min_loc = 0
    idx = 0

    while True:

        current = came_from[current]

        if current == start:
            break

        idx = idx + 1

        tmp = costmap[current]
        total_cost = total_cost + tmp
        if tmp < min_cost_in_path:
            min_cost_in_path = tmp
            min_loc = idx

        path.append(current)
    #path.append(start) # optional
    path.reverse() # optional

    min_loc = len(path) - min_loc - 1
    return path, min_loc, min_cost_in_path, total_cost

def reconstruct_path2(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    #path.append(start) # optional
    path.reverse() # optional
    return path

def a_star_search2(graph, start, goals, goals_time, heuristicgrid, grid):

    im = np.array(grid)

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0


    #new stuff
    path_lengths = {}
    path_lengths[start] = 0
    #viable_goals = list(goals)
    goal_idx = 0    #this is current goal we are going for
    path_count = 0

    best_path = []
    best_path_cost = 999999999

    curr_goal = tuple(goals_time[goal_idx, :2])

    #curr_goal = (992,999)
    sol_found = False

    nodes_expanded = 0

    found_goal = []
    
    while not frontier.empty():
        current = frontier.get()
        
        # what should heuristic be? (to first goal or closest goal)
        # going to first goal is optimistic
        # break when path len to goal == exact time at goal
        # how can we get the path length to current node efficiently?

        # do we do something , if we found osmething other than curr_goal
        # maybe remove it from goals to pursue?

        nodes_expanded = nodes_expanded + 1
        #print(current, path_lengths[current])
        if path_lengths[current] > goals_time[goal_idx, 2]:
            #print "skipping to next goal"

            if goal_idx < goals_time.shape[0]-1:
                goal_idx = goal_idx + 1
                curr_goal = tuple(goals_time[goal_idx, :2]) 

        if current in goals:

            for candidate in goals_time[np.all(goals_time[:,:2] == current, axis=1),:]:

                print candidate

            #     # if path_lengths < candidate[2]:
            #     #     if best_path_cost

            #     #this is the optimal solution
            #     if path_lengths[current] == candidate[2]:

            #         best_path = reconstruct_path2(came_from, start, curr_goal)
            #         best_path_cost = cost_so_far[current]
            #         print "best solution found"
            #         sol_found = True
            #         break
            print (current, cost_so_far[current], path_lengths[current])
            print ('curr_goal', curr_goal)
            found_goal = current
            break

        im[current] = 9999

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far:# or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + 100*heuristic(curr_goal, next)
                #priority = new_cost #+ 100*heuristicgrid[next]
                frontier.put(next, priority)
                came_from[next] = current

                path_lengths[next] = path_lengths[current] + 1


    # if best_path_len < exact goal time, fill states

    print ('nodes_expanded', nodes_expanded)
    
    # im = np.array(searcher.grid)
    # thepath = reconstruct_path2(came_from, start, found_goal)

    # for node in thepath:
    #     im[node] = 9999

    plt.matshow(im)
    plt.colorbar(orientation='vertical')
    plt.show()
    
    return came_from, cost_so_far

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


    mapz = GridWithWeights(searcher.dim, searcher.dim)
    mapz.weights = searcher.grid

    start = (0,0)
    #goal = (3,3)
    goal = tuple(searcher.target_traj[0])


    start_time = time.time()
    #came_from, cost_so_far = a_star_search(mapz, start, goal)
    #came_from, cost_so_far = dijkstra_search(mapz, start, goal)

    came_from, cost_so_far = a_star_search2(mapz, start,
                                 searcher.goals_set, searcher.goals,
                                 searcher.heuristicgrid, searcher.grid)

    print("time spent planning (sec):", time.time()-start_time)
    #print reconstruct_path(came_from, start, goal)

    # print "saving ..."
    # with open('search.pickle', 'wb') as handle:
    #     pickle.dump(cost_so_far, handle)
    # print "done saving!"

	# with open('filename.pickle', 'rb') as handle:
 #  		b = pickle.load(handle)


    # im = np.array(searcher.grid)
    # thepath = reconstruct_path2(came_from, start, goal)

    # for node in thepath:
    #     im[node] = 9999

    # plt.matshow(im)
    # plt.colorbar(orientation='vertical')
    # plt.show()
    # exit(0)




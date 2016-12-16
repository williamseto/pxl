#! /usr/bin/env python


from implementation import *
import numpy as np
from optparse import OptionParser
import matplotlib.pyplot as plt
from heapq import *
from functools import partial
import pickle
import time


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
        for t, pos in enumerate(self.target_traj):
            if manhattan(self.start, pos) <= t:
                self.goals = np.vstack((self.goals, pos+[t]))

        # print self.goals
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

def dijkstra_search2(graph, start, goal, grid):

    im = np.array(grid)

    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    nodes_expanded = 0
    while not frontier.empty():
        current = frontier.get()
        
        nodes_expanded = nodes_expanded + 1
        im[current] = 9999
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    print nodes_expanded

    # im[(0,0)] = 0
    # print im
    # plt.matshow(im)
    # plt.colorbar(orientation='vertical')
    # plt.show()
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

    # im = np.array(searcher.grid)
    # for node in searcher.goals:
    #     im[node[0], node[1]] = 9999

    # plt.matshow(im)
    # plt.colorbar(orientation='vertical')
    # plt.show()
    # exit(0)

    start_time = time.time()
    #came_from, cost_so_far = a_star_search(mapz, start, goal)
    #came_from, cost_so_far = dijkstra_search2(mapz, start, goal, searcher.grid)
    came_from, cost_so_far = dijkstra_search(mapz, start, goal)

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

    im = np.zeros([1000,1000])
    for key, value in cost_so_far.iteritems():
        im[key] = value

    plt.matshow(im)
    plt.colorbar(orientation='vertical')
    plt.show()
    exit(1)

    start_time = time.time()
    best_cost = 9999999999
    best_path = []
    best_min_cost = 0
    time_diff = 0

    for tgoal in searcher.goals:

        path, min_loc, min_cost, total_cost = reconstruct_path(came_from, start, tuple(tgoal[:2]), searcher.grid)

        if len(path) <= tgoal[2]:
            curr_cost = total_cost + (tgoal[2] - len(path))*min_cost

            #print (tgoal, len(path), total_cost, curr_cost, min_cost, min_loc)

            if curr_cost < best_cost:
                best_path = path
                best_cost = curr_cost
                best_min_cost = min_cost
                time_diff = int(tgoal[2] - len(path))


    # wrong
    # for i in range(time_diff):
    #     best_path.insert(min_loc, path[min_loc])

    # print best_path
    print best_cost

    im = np.array(searcher.grid)
    for node in best_path:
        im[node[0], node[1]] = 9999

    plt.matshow(im)
    plt.colorbar(orientation='vertical')
    plt.show()


    print("time spent choosing best route (ms):", (time.time()-start_time)*1000.0)


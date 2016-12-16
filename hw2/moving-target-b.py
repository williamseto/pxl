#! /usr/bin/env python

import numpy as np
from optparse import OptionParser
import matplotlib.pyplot as plt
from heapq import *
import time
import matplotlib.pyplot as plt

from util import *


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

def SMHAstar(problem):
    im = np.array(problem.grid)


    start = problem.start
    path_lengths = {}
    path_lengths[start] = 0

    w1 = 60
    w2 = 5

    bp = {}
    g = {}
    bp[start] = None
    g[start] = 0

    open_lists = []

    #find closest/furthese point on trajectory
    goal_list = list(problem.goals_set)
    goal_dists = map(lambda x : manhattan(x, start), goal_list)
    _, min_idx = min((val, idx) for (idx, val) in enumerate(goal_dists))
    _, max_idx = max((val, idx) for (idx, val) in enumerate(goal_dists))

    #heuristics are: to closest point, furthest point, and last point in trajectory
    #anchor is last point (conservative)
    h0 = lambda x : manhattan(x, tuple(problem.target_traj[-1])) 
    h1 = lambda x : manhattan(x, goal_list[min_idx])
    h2 = lambda x : manhattan(x, goal_list[max_idx])

    h_list = [h0, h1, h2]
    terminate = False

    # one for anchor search, one for all other searches
    exp_lists = [set(), set()]

    #initialize all search queues
    for i in range(3):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        open_lists.append(frontier)

    nodes_expanded = 0

    found_goal = None

    while not open_lists[0].empty():

        for i in range(1,3):

            # if OPEN_i.minkey <= w2*OPEN_0.minkey
            if open_lists[i].minkey() <= w2*open_lists[0].minkey():
                s = open_lists[i].get()
                exp_idx = i
            else:
                s = open_lists[0].get()
                exp_idx = 0

            if s in problem.goals_set:

                #also check that our path length <= target timestep
                for candidate in problem.goals[np.all(problem.goals[:,:2] == s, axis=1),:]:
                    if path_lengths[s] <= candidate[2]:
                        terminate = True
                        found_goal = candidate
                        break
                break

            nodes_expanded = nodes_expanded + 1
            im[s] = 9999

            #expand(s)
            for s_prime in problem.get_successors(s):
                new_cost = g[s] + problem.grid[s_prime]
                if s_prime not in g or new_cost < g[s_prime]:
                    g[s_prime] = new_cost
                    bp[s_prime] = s
                    path_lengths[s_prime] = path_lengths[s] + 1

                    # if s' has not been expanded in the anchor search
                    if s_prime not in exp_lists[0]:
                        p0 = new_cost + w1*h_list[0](s_prime)
                        open_lists[0].put(s_prime, p0)

                        # if s' has not been expanded in any inadmissible search
                        if s_prime not in exp_lists[1]:
                            for j in range(1,3):
                                p_i = new_cost + w1*h_list[j](s_prime)
                                if p_i <= w2*p0:
                                    open_lists[j].put(s_prime, p_i)

            if exp_idx == 0:
                exp_lists[0].add(s)
            else:
                exp_lists[1].add(s)
            #end expand


        if terminate:
            #print found_goal
            print "nodes expanded", nodes_expanded
            break

    return bp, found_goal, g

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

    start_time = time.time()

    came_from, found_goal, gvalues = SMHAstar(searcher)

    #now reconstruct path, filling in spots in the path if path length < time
    current = tuple(found_goal[:2])
    path = [current]

    tmp = searcher.grid[current]
    min_cost_in_path = tmp
    min_idx = 0
    idx = 0
    min_node = None

    while True:

        current = came_from[current]

        if current == searcher.start:
            break

        idx = idx + 1

        tmp = searcher.grid[current]
        if tmp < min_cost_in_path:
            min_cost_in_path = tmp
            min_idx = idx
            min_node = current

        path.append(current)

    path.reverse()

    min_idx = len(path) - min_idx - 1

    #total cost is gvalue plus time_difference*min_cost_in_path
    time_difference = found_goal[2]-len(path)

    #fill in steps if necessary
    for t in range(time_difference):
        path.insert(min_idx, min_node)
    total_cost = gvalues[tuple(found_goal[:2])] + (time_difference)*min_cost_in_path

    
    #now print path
    print total_cost
    print searcher.start
    for node in path:
        print node
    print("time spent planning (sec):", time.time()-start_time)

    # im = np.array(searcher.grid)

    # for node in thepath:
    #     im[node] = 9999

    # plt.matshow(im)
    # plt.colorbar(orientation='vertical')
    # plt.show()





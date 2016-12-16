#! /usr/bin/env python

import numpy as np
from optparse import OptionParser
import matplotlib.pyplot as plt
from heapq import *
from util import *
import time
import cPickle as pickle


def dijkstra_search(problem, start):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    path_lengths = {}
    path_lengths[start] = 0

    #minor optimizations
    cost_grid = problem.grid
    get_successors = problem.get_successors
    frontier_get = frontier.get
    frontier_put = frontier.put
    frontier_empty = frontier.empty

    while not frontier_empty():
        current = frontier_get()
        
        for next in get_successors(current):
            new_cost = cost_so_far[current] + cost_grid[next]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier_put(next, priority)
                came_from[next] = current

                path_lengths[next] = path_lengths[current] + 1

    return came_from, cost_so_far, path_lengths

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

    print "begin planning .."
    start_time = time.time()
    _, gscore_2d, path_lengths = dijkstra_search(searcher, searcher.start)


    #begin 3D a-star
    frontier = PriorityQueue()
    came_from = {}
    cost_so_far = {}

    min_cost = {}

    nodes_expanded = 0

    goal = (0,0,0)

    #push all traj positions
    for node in searcher.goals_time_set:
        frontier.put(node, gscore_2d[node[:2]] )
        came_from[node] = None
        cost_so_far[node] = searcher.grid[node[:2]]
        min_cost[node] = searcher.grid[node[:2]]
    
    while not frontier.empty():

        nodes_expanded = nodes_expanded + 1
        #print frontier.elements[0], nodes_expanded

        current = frontier.get()

        if current == goal:
            #print (current, cost_so_far[current] - searcher.grid[(0,0)])
            break

        # if nodes_expanded % 10000 == 0:
        #     print current
        
        if searcher.grid[current[:2]] <= min_cost[current]:
            minima = 1
        else:
            minima = 0

        for next in searcher.get_successors_3D(current, minima):
            new_cost = cost_so_far[current] + searcher.grid[next[:2]]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost

                #add an extra subtraction since we double count costmap value
                priority = new_cost + (gscore_2d[next[:2]]) - searcher.grid[next[:2]] #+ abs(node[2] - path_lengths[node[:2]]-1)
                frontier.put(next, priority)
                came_from[next] = current

                if searcher.grid[next[:2]] < min_cost[current]:
                    min_cost[next] = searcher.grid[next[:2]]
                else:
                    min_cost[next] = min_cost[current]

    print "nodes expanded:", nodes_expanded
    print("time spent planning (sec):", time.time()-start_time)

    path = []
    #subtract cost at (0,0) since we were planning backwards
    print cost_so_far[current] - searcher.grid[(0,0)]
    while current != None:
        path.append(current)
        print current[:2]
        current = came_from[current]
    #print path

#! /usr/bin/env python

import numpy as np
from optparse import OptionParser
from scipy.sparse.csgraph import csgraph_from_dense, dijkstra

def euclidean(s1, s2):
    return np.sqrt((s1[0]-s2[0])**2 + (s1[1]-s2[1])**2)

if __name__ == '__main__':

    usage = "usage: %prog filename"
    parser = OptionParser(usage=usage)
    (options, args) = parser.parse_args()

    if not args:
        parser.print_help()
        exit(0)

    with open(args[0]) as problemfile:    
        lines = problemfile.read().splitlines()

   	idx = 0
    while lines[idx] != "0":

    	#construct waypoints
    	dim = int(lines[idx])
    	idx = idx + 1
    	waypoints = np.empty((dim,3))
    	for k in range(dim):
            waypoints[k,:] = map(int, lines[idx+k].split(" "))
        idx = idx + dim

        #augment waypoints with (0,0) & (100,100)
        waypoints = np.vstack(([0,0,0], waypoints, [100,100,0]))

        n = waypoints.shape[0]
        graph = np.zeros([n, n])
        for i in range(n):
        	for j in range(i+1,n):
        		#time to reach wpt + any penalty + 10 seconds to load
        		graph[i,j] = euclidean(waypoints[i,:2], waypoints[j,:2])/2 + sum(waypoints[i+1:j, 2]) + 10

        sparse_graph = csgraph_from_dense(graph)
        (dist, preds) = dijkstra(sparse_graph, return_predecessors=True)

        print round(dist[0,-1], 3)

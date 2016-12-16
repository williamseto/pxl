import numpy as np
import time
import heapq

def manhattan(s1, s2):
    return abs(s1[0]-s2[0]) + abs(s1[1]-s2[1])


class Searcher():
    def __init__(self, filelines):

        print "Beginning to read problem file .."
        t1 = time.time()        
        self.dim = int(filelines[1])
        self.start = tuple(map(int, filelines[3].split(",")))
        i = 5
        self.target_traj = []
        self.grid = np.zeros([self.dim, self.dim])
        while filelines[i] != "B":
            self.target_traj.append(map(int,filelines[i].split(",")))
            i = i + 1
        i = i + 1
        for k in range(self.dim):
            self.grid[k,:] = map(int, filelines[i+k].split(","))

        print "Finished reading problem file in", time.time()-t1, "seconds"
        #prune feasible goals from the trajectory
        self.goals = np.empty((0,3), dtype=np.int)
        self.goals_time_set = set()
        self.goals_set = set()
        for t, pos in enumerate(self.target_traj):
            if manhattan(self.start, pos) <= t:
                self.goals = np.vstack((self.goals, pos+[t]))
                self.goals_time_set.add(tuple(pos) + (t,))
                self.goals_set.add(tuple(pos))

    def in_bounds(self, state):
        (x, y) = state
        return 0 <= x < self.dim and 0 <= y < self.dim

    def get_successors(self, state):
        (x, y) = state
        succs = set([(x+1, y), (x, y-1), (x-1, y), (x, y+1)])
        succs = filter(self.in_bounds, succs)
        return succs

    def in_bounds_3D(self, state):
        (x, y, t) = state
        return 0 <= x < self.dim and 0 <= y < self.dim and t >= (x+y)

    def get_successors_3D(self, state, minima):
        (x, y, t) = state

        # only reason to stay at a current state is if it's low cost
        if minima == 1:
            succs = [(x+1, y, t-1), (x, y-1, t-1), (x-1, y, t-1), (x, y+1, t-1), \
                    (x, y, t-1)]
        else:
            succs = [(x+1, y, t-1), (x, y-1, t-1), (x-1, y, t-1), (x, y+1, t-1)]
        succs = filter(self.in_bounds_3D, succs)
        return succs


class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

    def minkey(self):
        return self.elements[0][0]
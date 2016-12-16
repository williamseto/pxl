#! /usr/bin/env python

import numpy as np
import json
from optparse import OptionParser


class Representation1():
    def __init__(self):
        pass

    @staticmethod
    def get_start_state(ships):
        start_state = []
        for i in range(len(ships)):
            start_state.append((ships[i]["position"][0],
                                ships[i]["position"][1]))
        return start_state

    @staticmethod
    def state_not_closed(state, closedlist):
        return state not in closedlist

    @staticmethod
    def is_goal_state(gamestate):
        if gamestate[-1] == (3,3):
            return True
        else:
            return False

    @staticmethod
    def get_ship_successor(gamestate, i, color):

        ship = gamestate[i]

        succs = []
        left = []
        right = []
        up = []
        down = []
        for n in [x for x in range(len(gamestate)) if x != i]:

            #left; ship[i] is same row and right of ship[n]
            if ship[1] > gamestate[n][1] and ship[0] == gamestate[n][0]:

                #print('Leftcheck', i,n, gamestate[i], gamestate[n])
                if not left:
                    left = [list(gamestate), [color, "Left"]]
                    left[0][i] = (ship[0], gamestate[n][1] + 1)
                else:
                    #check if our movement should be shorter
                    if gamestate[n][1] >= left[0][i][1]:
                        left[0][i] = (ship[0], gamestate[n][1] + 1)

            #right; ship[i] is same row and left of ship[n]
            if ship[1] < gamestate[n][1] and ship[0] == gamestate[n][0]:

                if not right:
                    right = [list(gamestate), [color, "Right"]]
                    right[0][i] = (ship[0], gamestate[n][1] - 1)
                else:
                    #check if our movement should be shorter
                    if gamestate[n][1] <= right[0][i][1]:
                        right[0][i] = (ship[0], gamestate[n][1] - 1)  

            #up; ship[i] is col row and below ship[n]
            if ship[0] > gamestate[n][0] and ship[1] == gamestate[n][1]:

                if not up:
                    up = [list(gamestate), [color, "Up"]]
                    up[0][i] = (gamestate[n][0] + 1, ship[1])
                else:
                    #check if our movement should be shorter
                    if gamestate[n][0] >= up[0][i][0]:
                        up[0][i] = (gamestate[n][0] + 1, ship[1])  

            #down; ship[i] is same col and above ship[n]
            if ship[0] < gamestate[n][0] and ship[1] == gamestate[n][1]:

                #print('Downcheck', i,n, gamestate[i], gamestate[n])
                if not down:
                    down = [list(gamestate), [color, "Down"]]
                    down[0][i] = (gamestate[n][0] - 1, ship[1])
                else:
                    #check if our movement should be shorter
                    if gamestate[n][0] <= down[0][i][0]:
                        down[0][i] = (gamestate[n][0] - 1, ship[1])

        if left and left[0][i] != ship:
            #print ("left", left)
            succs.append(left)
        if right and right[0][i] != ship:
            succs.append(right)
        if up and up[0][i] != ship:
            succs.append(up)
        if down and down[0][i] != ship:
            succs.append(down)

        return succs
        
    @staticmethod
    def get_successors(gamestate, ships):

        succs = []
        for i in range(len(gamestate)):
            succs.extend(Representation1.get_ship_successor(gamestate, i, ships[i]["color"]))
        return succs

class Representation2():
    def __init__(self):
        pass

    @staticmethod
    def get_start_state(ships):
        state = np.zeros([5,5])
        for i in range(len(ships)):
            state[ships[i]["position"][0]-1,
                  ships[i]["position"][1]-1] = i+1
        return state

    @staticmethod
    def state_not_closed(state, closedlist):
        return not any((state == el).all() for el in closedlist)

    @staticmethod
    def is_goal_state(gamestate):
        return gamestate[2,2] == np.max(gamestate)

    @staticmethod
    def get_successors(gamestate, ships):

        succs = []

        for pos, val in np.ndenumerate(gamestate):

            if val != 0:
                color = ships[int(val-1)]["color"]

                #left
                if pos[1] > 1 and gamestate[pos[0], pos[1]-1] == 0:
                    for i in range(1,4):
                        col = pos[1]-i
                        if col <= 0:
                            break
                        if (gamestate[pos[0], col-1] != 0):
                            succ = np.copy(gamestate)
                            succ[pos[0], col] = val
                            succ[pos[0], pos[1]] = 0
                            succs.append([succ, [color, 'Left']])
                            break

                #right
                if pos[1] < 3 and gamestate[pos[0], pos[1]+1] == 0:
                    for i in range(1,4):
                        col = pos[1]+i
                        if col >= 4:
                            break
                        if (gamestate[pos[0], col+1] != 0):
                            succ = np.copy(gamestate)
                            succ[pos[0], col] = val
                            succ[pos[0], pos[1]] = 0
                            succs.append([succ, [color, 'Right']])
                            break

                #up
                if pos[0] > 1 and gamestate[pos[0]-1, pos[1]] == 0:
                    for i in range(1,4):
                        row = pos[0]-i
                        if row <= 0:
                            break
                        if (gamestate[row-1, pos[1]] != 0):
                            succ = np.copy(gamestate)
                            succ[row, pos[1]] = val
                            succ[pos[0], pos[1]] = 0
                            succs.append([succ, [color, 'Up']])
                            break

                #down
                if pos[0] < 3 and gamestate[pos[0]+1, pos[1]] == 0:
                    for i in range(1,4):
                        row = pos[0]+i
                        if row >= 4:
                            break
                        if (gamestate[row+1, pos[1]] != 0):
                            succ = np.copy(gamestate)
                            succ[row, pos[1]] = val
                            succ[pos[0], pos[1]] = 0
                            succs.append([succ, [color, 'Down']])
                            break

        return succs        


if __name__ == '__main__':

    usage = "usage: %prog [options] arg1 arg2"
    parser = OptionParser(usage=usage)
    parser.add_option("-f", "--file", dest="filename",
                  help="the puzzle file", metavar="FILE")
    parser.add_option("-r", type="int", dest="num",
                        help="1 for representation1, 2 for representation2")

    (options, args) = parser.parse_args()

    if options.num is None or options.filename is None:
        parser.print_help()
        exit(0)

          
    with open(options.filename) as puzzle_file:    
        puzzle_data = json.load(puzzle_file)

    ships = puzzle_data["ships"]

    for i in range(len(ships)):
        #fix unicode representation
        ships[i]["color"] = str(ships[i]["color"])


    if options.num == 2:
        print("choosing representation 2")
        Problem = Representation2()
    else:
        print("choosing representation 1")
        Problem = Representation1()


    start_state = Problem.get_start_state(ships)
    closed = []

    fringe = []
    fringe.append([start_state, []])

    states_expanded = 0
    #run dfs
    while(1):
        curr = fringe.pop()

        states_expanded = states_expanded + 1
        curr_state = curr[0]
        curr_path = curr[1]

        #print ("curr state", curr)

        #check goal state
        if Problem.is_goal_state(curr_state):

            print("SUCCESS, states expanded: ", states_expanded)
            print(curr_path)
            exit(1)
            #return curr_path

        if Problem.state_not_closed(curr_state, closed):
            closed.append(curr_state)

            successors = Problem.get_successors(curr_state, ships)

            for state in successors:

                #print([state[0], curr_path.__add__([state[1]])])
                fringe.append([state[0], curr_path.__add__([state[1]])])

        #s = raw_input("press enter to continue")

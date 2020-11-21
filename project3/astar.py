# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 10] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)
                
goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def compute_path(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up    
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))], 
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    
    
    x = start[0]
    y = start[1]
    theta = start[2]
    
    h = heuristic[x][y]
    g = 0
    f = g+h
    
    open_set.put(start, Value(f=f,g=g))

    # your code: implement A*
   
    # I just modified how the walls and traversable cells appear. So that it's easier to see.
    # ☐ indicates a wall
    # O indicates a traversable path
    for i in range(0,len(path)):
        for j in range(0,len(path[0])):
            if(grid[i][j] == 1):
                
                path[i][j] = '☐'
            else:
                path[i][j] = 'O'
            
    
    # Represent goal as *
    path[goal[0]][goal[1]] = '*'
    
    # A dictionary that uses a state(grid row, grid col, orientation) as an index to store its parent and its f and g values.
    # This helps us keep track of g values as we expand the nodes.    
    cost_set = {}
    cost_set[start]=([None],0,0)
   
    print("Cost: ",cost)
    while open_set:
        node = open_set.pop()
        closed_set.add(node[0])
        if node[0] == goal:
            break
            
            
        
        p = node[0]        
        children = []
        
        #Iterate over all potential children of a node
        for a in action:
            x = -1
            y = -1
            
            #Calculate new orientation
            newOrient = p[2] + a
            if(newOrient < 0):
                newOrient = 3
            elif(newOrient > 3):
                newOrient = newOrient%4
                
            x = p[0] + forward[newOrient][0]
            y = p[1] + forward[newOrient][1]
            
            #Only compute if the child is valid and traversable 
            if( x >= 0 and  x < len(grid) and y >= 0 and y < len(grid[0]) and grid[x][y] == 0):                
                child = (x,y,newOrient)
                h = heuristic[child[0]][child[1]]
                g = cost_set[p][2] + cost[a+1] 
                f = g+h
                if(child not in open_set and child not in closed_set):
                    open_set.put(child, Value(f=f,g=g))
                    cost_set[child] = (p,f,g)
                elif(g < cost_set[child][2]):
                    open_set.remove(child)
                    open_set.put(child, Value(f=f,g=g))
                    cost_set[child] = (p,f,g)
        
    
    # Starting from the goal, backtrack through the solution the construct the path
    path_node = goal   
    while path_node!= start:
        curr_node = path_node
        path_node = cost_set[path_node][0]
        x = curr_node[0] - path_node[0]
        y = curr_node[1] - path_node[1]
        if([x,y] in forward):            
            a = curr_node[2] - path_node[2]
            if(abs(a) == 3):
                a = -int(a/abs(a))            
            if(a in action):                    
                path[path_node[0]][path_node[1]] = action_name[a+1]
       
        
   
                
    
    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    return path, closed_set


def compute_path_djikstra(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up    
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))], 
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    
    
    x = start[0]
    y = start[1]
    theta = start[2]
    
    h = heuristic[x][y]
    g = 0
    f = g+h
    
    open_set.put(goal, Value(f=g,g=g))

    # your code: implement Djikstra
    
    
   
    
    for i in range(0,len(path)):
        for j in range(0,len(path[0])):
            if(grid[i][j] == 1):
                
                path[i][j] = '☐'
            else:
                path[i][j] = 'O'
            
            
    path[goal[0]][goal[1]] = '*'
    cost_set = {}
    cost_set[goal]=([None],0,0)
    
    
    
    print("Cost: ",cost)
    # For Djikstra's, we start from the goal, and expand all the nodes and compute the shortest path from each expanded node to the goal.
    while open_set:
        node = open_set.pop()
        closed_set.add(node[0])                    
            
        
        c = node[0]        
        children = []
        for a in action:
            
            x = -1
            y = -1
            orient = c[2]
            x = c[0] - forward[orient][0]
            y = c[1] - forward[orient][1]
            
            if( x >= 0 and  x < len(grid) and y >= 0 and y < len(grid[0]) and grid[x][y] == 0):            
                preorient = orient - a
                if(preorient < 0):
                    preorient = 3
                elif(preorient > 3):
                    preorient = preorient%4                             

                parent  = (x,y,preorient)
                
                g = cost_set[c][2] + cost[a+1]
                
                if(parent not in open_set and parent not in closed_set):
                    open_set.put(parent, Value(f=g,g=g))
                    cost_set[parent] = (c,g,g)
                if(g < cost_set[parent][2]):
                    open_set.remove(parent)
                    open_set.put(parent, Value(f=g,g=g))
                    cost_set[parent] = (c,g,g)

        
        
    
    # Logic is similar to A* but here we begin from the start node and compute the path till the goal.
    path_node = start    
    while path_node!= goal:
        curr_node = path_node
        path_node = cost_set[path_node][0]

        x = path_node[0] - curr_node[0]
        y = path_node[1] - curr_node[1]
        if([x,y] in forward):
            a = path_node[2] - curr_node[2]
            if(abs(a) == 3):
                a = -int(a/abs(a))             
            if(a in action):                    
                path[curr_node[0]][curr_node[1]] = action_name[a+1]
    
    
    
    
       
        
   
                
    
    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    return path, closed_set

if __name__ == "__main__":
    
    choice = input("Enter 0 for A* and 1 for Djikstra ")
    if(choice == '0'):
        path,closed=compute_path(grid, init, goal, cost, heuristic)
    elif(choice == '1'):
        
        path,closed=compute_path_djikstra(grid, init, goal, cost, heuristic)
    else:
        print("wrong input")
        
    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")    
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""
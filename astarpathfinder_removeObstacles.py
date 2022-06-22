import numpy
from heapq import *
import random
from random import sample
import math
# heap used to implement priorityqueue

# to return expected distance. Since diagonal movement is possible, 
# euclidean distance is considered here instead of manhattan distance
def heuristic(a, b):
    return numpy.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def addobstacles(shape, start, end, obstacles):
    # new_obstacles = [tuple(i) for i in sample(tuple(numpy.argwhere((array==0))),20)]    
    invalid_choices =[start,end]+obstacles
    valid_choices = [(i,j) for i, j in numpy.ndindex(shape) if (i,j) not in invalid_choices]
    new_obstacles = [tuple(i) for i in sample(valid_choices,20)]
    obstacles = obstacles + new_obstacles
    return obstacles

def route(array, start, end, obstacles):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
# set to store elements that were already covered/need not be considered again
    close_set = set()
# to backtrack the path covered- contains all covered routes in each iteration
    path = {}
# distance from start
    gscore = {start:0}
# total expected distance to end
    fscore = {start:heuristic(start, end)}
    open_heap = []

    heappush(open_heap, (fscore[start], start))
    
    while open_heap:
        # extract neighbour with least f score
        current = heappop(open_heap)[1]

        if current == end:
            shortest_route = []
            while current in path:
                shortest_route.append(current)
                current = path[current]
            no_obstacles = fscore[end]/1000
            if no_obstacles>=1:
                print("Unable to reach delivery point")
                removables = list(set(obstacles) & set(shortest_route))
                print("Need to remove",math.floor(no_obstacles),"obstacle(s) at ",removables," to find an optimal path")
            return shortest_route[::-1]
        #add current position to covered set
        close_set.add(current)
        #loop over neighbors to find their g scores
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j      
            expected_gscore = gscore[current] + heuristic(current, neighbor)
            # eliminate invalid positions and obstacles
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        expected_gscore = 1000 + gscore[current] + heuristic(current, neighbor)
                else:
                    continue
            else:
                continue
            
            # neighbor already considered but expected score via updated route is larger than score earlier found
            if neighbor in close_set and expected_gscore >= gscore.get(neighbor, 0):
                continue
            
            # expected score via updated route is smaller than score earlier found or found new position not already
            # in open set
            if  expected_gscore < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_heap]:
                path[neighbor] = current
                gscore[neighbor] = expected_gscore
                fscore[neighbor] = expected_gscore + heuristic(neighbor, end)
                heappush(open_heap, (fscore[neighbor], neighbor))
    return False
    
nmap = numpy.zeros((10,10))
# assuming obstacles are at (7,7) and (7,8) according to diagram instead of phase 1 text.
obstacles = [(9,7),(8,7),(7,7),(7,8),(7,9),(6,9),(6,8),(6,7),(6,6), (7,6),(8,6),(9,6)]
start = (0,0)
end = (9,9)
# obstacles = addobstacles(nmap.shape, start, end, obstacles)   
for obs in obstacles:
    nmap[obs] =1
print("Obstacles at =" , obstacles)
route = route(nmap, start, end, obstacles)

if route:
    route = [start] + route
    print("Path =" , route)
    print("Number of steps taken from ", start ," =" , len(route)-1)
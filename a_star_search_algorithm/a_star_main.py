#Authors: Daniel Eisenbach and Tao Chen
#Course: ROB 456
#Date: 11/19/16

#g value: cost of getting from start node to that node
#h value: heuristic value calculated using pythagoreans theorem on positon of node and goal
#f value: sum of g and h values of a node

import heapq
import csv
import sys

start = [1, 1]
goal = [20, 20]

#pull world info from csv file
world = []
with open('./world.csv', 'rU') as f:
  reader = csv.reader(f)
  for row in reader:
    world.append(row)
 
class node:
  def __init__(self, pos):
    self.pos = pos 
    #heuristic calculated with pythagoreans theorem using position of node and goal
    self.h = ((goal[1] - pos[1])**2 + (goal[0] - pos[0])**2)**(0.5)   

  parent = []  
  h = float('inf')  #heuristic
  g = float('inf')  #the cost of getting from the start node to that node
  f = g + h

  def set_g(self, g):  #function to update g value for a node
    self.g = g
    self.f = self.g + self.h  #update f value based upon new g value

  def info(self):
    print 'pos: {0}'.format(self.pos)  #print node's position
    print 'parent: {0}'.format(self.parent)  #print position of node's parent
    print 'f: {0}'.format(self.f)  #print f value of node = g + h

#initialize set of nodes to fill [1-20] X [1-20] grid
nodes = []
for i in range(1, 21):
  nodes.append([])
  for j in range(1, 21):
    nodes[i-1].append(node([i, j]))

#keep track of nodes that need to be evaluated
openSet = []  #nodes that need need to be evaluated
closedSet = []  #nodes that have been evaluated
path = []  #nodes that are part of the path

#intialize starting node
start_node = nodes[start[0] - 1][start[1] - 1]  #starting node
start_node.set_g(0)  #initialize g value for start to zero
start_node.parent = start  #initialize parent of start to be start
heapq.heappush(openSet, (start_node.f, start_node))  #add starting node to open set for evaluation

while(openSet): 
  current = heapq.heappop(openSet)[1]  #node in open set with the lowest f value
  if current.pos == goal:  #if the goal has been reached, break out of the loop and reconstruct path
    print ""
    print 'Goal reached!'
    print "Steps: {0}".format(current.g)
    #reconstruct path
    break
  
  closedSet.append(current)  #add the current node to the closed set (evaluated nodes)
  x = current.pos[0] - 1
  y = current.pos[1] - 1
  tentative_gScore = current.g + 1

  #FOR ALL NEIGHBORS:
  #Check if node not in closed set, if node has smaller g cost than previously found, and if node is in the map.
  #If a node is not in the openSet, add it to it.
  #If this is a better path, aka lower g value, update the parent and the g value

  #check top neighbor
  if (y - 1) >= 0 and world[y-1][x] == '0' and nodes[x][y-1] not in closedSet and tentative_gScore < nodes[x][y-1].g:
    if nodes[x][y-1] not in openSet:
      heapq.heappush(openSet, (nodes[x][y-1].f, nodes[x][y-1]))
    nodes[x][y-1].parent = current.pos  #update parent
    nodes[x][y-1].set_g(tentative_gScore)  #update g value
    heapq.heappush(openSet, (nodes[x][y-1].f, nodes[x][y-1]))

  #check bottom neighbor
  if (y + 1) < 20 and world[y+1][x] == '0' and nodes[x][y+1] not in closedSet and tentative_gScore < nodes[x][y+1].g:
    if nodes[x][y+1] not in openSet:
      heapq.heappush(openSet, (nodes[x][y+1].f, nodes[x][y+1]))
    nodes[x][y+1].parent = current.pos  #update parent
    nodes[x][y+1].set_g(tentative_gScore)  #update g value
    heapq.heappush(openSet, (nodes[x][y+1].f, nodes[x][y+1]))

  #check left neighbor
  if (x - 1) >= 0 and world[y][x-1] == '0' and nodes[x-1][y] not in closedSet and tentative_gScore < nodes[x-1][y].g:
    if nodes[x-1][y] not in openSet:
      heapq.heappush(openSet, (nodes[x-1][y].f, nodes[x-1][y]))
    nodes[x-1][y].parent = current.pos  #update parent
    nodes[x-1][y].set_g(tentative_gScore)  #update g value
    heapq.heappush(openSet, (nodes[x-1][y].f, nodes[x-1][y]))

  #check right neighbor
  if (x + 1) < 20 and world[y][x+1] == '0' and nodes[x+1][y] not in closedSet and tentative_gScore < nodes[x+1][y].g:
    if nodes[x+1][y] not in openSet:
      heapq.heappush(openSet, (nodes[x+1][y].f, nodes[x+1][y]))
    nodes[x+1][y].parent = current.pos  #update parent
    nodes[x+1][y].set_g(tentative_gScore)  #update g value
    heapq.heappush(openSet, (nodes[x+1][y].f, nodes[x+1][y]))

#reconstruct path from start towards goal by looking at the parent of each consecutive node attached to the goal's parent
cur_x = goal[0]
cur_y = goal[1]
while (cur_x != start[0] or cur_y != start[1]):
  parent_temp = nodes[cur_x - 1][cur_y - 1].parent
  path.append(nodes[cur_x - 1][cur_y - 1].pos)
  cur_x = parent_temp[0]
  cur_y = parent_temp[1]

path.append(start_node.pos)  #add starting node to path

#print out final path
print ""
print "Final Path: "
for node in path:
  world[node[1]-1][node[0]-1] = '2'
  print node

for row in world:
    for cell in row:
      if cell == '1':
        sys.stdout.write("X ")
      else:
        sys.stdout.write("O ")
    print
print "============================"

for row in world:
    for cell in row:
      if cell == '1':
        sys.stdout.write("X ")
      elif cell == '0':
        sys.stdout.write("O ")
      else:
        sys.stdout.write("= ")
    print

while True:
	pass
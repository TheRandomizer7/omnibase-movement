#!/usr/bin/env python3

import matplotlib.pyplot as plt
import random
import math
import rospy

from shapely.geometry import Point, Polygon, LineString
from movement_nodes.msg import Coordinates

#Functions
def CheckIfValidConnection(point, node_to_connect):
    is_valid = True
    #slope_of_line = (node_to_connect.y_coord - point[1])/(node_to_connect.x_coord - point[0])
    #y_intercept = node_to_connect.y_coord - slope_of_line * node_to_connect.x_coord
#
    #for obj in obstacles:
    #    center_x = obj[0]
    #    center_y = obj[1]
    #    distance_from_center = abs(slope_of_line * center_x - center_y + y_intercept)/((1 + slope_of_line ** 2) ** 0.5)
    #    #print(distance_from_center)
    #    if(distance_from_center < radius_of_obstacles):
    #        print(str(obj) + " (" + str(node_to_connect.x_coord) + ", " + str(node_to_connect.y_coord) +"), (" + str(point[0]) + ", " + str(point[1]) + ")" + " y = " + str(slope_of_line) +"x + " + str(y_intercept))
    #        is_valid = False
    #        break

    return is_valid

def CheckIfValidPoint(point):
    is_valid = True
    if(len(obstacles) != 0):
        for obj in obstacles:
            radius = ((point[0] - obj[0]) ** 2 + (point[1] - obj[1]) ** 2) ** 0.5
            if(radius < radius_of_obstacles):
                is_valid = False
                break
    else:
        is_valid = False
    
    return is_valid

def ObstacleCoordinates(msg):
    global obstacles
    array = []
    i = 0
    while(i < len(msg.x_coords)):
        array.append([msg.x_coords[i], msg.y_coords[i]])
        i = i + 1
    obstacles.clear()
    obstacles = array


#Classes
class Figures:

    def __init__(self, coord_list):
        self.coord_list = coord_list
        self.polygon = Polygon(coord_list)

class Nodes:

    def __init__(self, x_coord, y_coord, neighbour_indexes, parent_index, cost, self_index):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.neighbour_indexes = neighbour_indexes
        self.parent_index = parent_index
        self.cost = cost
        self.self_index = self_index

#Variables
point_of_origin = (0, 0) #start point
bounds_of_plane = [0, 0, 6, 6]
fixed_distance = 0.25
total_nodes = 4000
counter = 0
found_goal = False
radius_of_obstacles = 0.6

global obstacles
obstacles = []
#obstacles.append(Figures([[10, 70], [20, 80], [40, 80], [40, 60], [20, 60], [10, 70]]))
#obstacles.append(Figures([[60, 80], [65, 80], [65, 20], [60, 20], [60, 80]]))
#obstacles.append(Figures([[20, 30], [30, 10], [10, 10], [20, 30]]))

goal = (6, 6)

nodes_in_tree = []
nodes_in_tree.append(Nodes(point_of_origin[0], point_of_origin[1], [], 0, 0, 0))

rospy.init_node('path_planner')
pub = rospy.Publisher('target_path', Coordinates, queue_size = 1)
obstacle_sub = rospy.Subscriber('obstacle_coords', Coordinates, ObstacleCoordinates)

while (len(obstacles) == 0):
    print("empty")
print("done")
#Main loop of PRM
for i in range(total_nodes):
    rand_point = [random.uniform(bounds_of_plane[0], bounds_of_plane[2]), random.uniform(bounds_of_plane[1], bounds_of_plane[3])]

    if(not CheckIfValidPoint(rand_point)):
        continue

    nodes_in_tree.append(Nodes(rand_point[0], rand_point[1], [], 0, 0, len(nodes_in_tree)))
     
    for i in range(len(nodes_in_tree) - 1):
        dist = math.sqrt((nodes_in_tree[i].x_coord - rand_point[0]) ** 2 + (nodes_in_tree[i].y_coord - rand_point[1]) ** 2)

        if(dist <= fixed_distance and CheckIfValidConnection(rand_point, nodes_in_tree[i])):
            nodes_in_tree[i].neighbour_indexes.append(len(nodes_in_tree) - 1)
            nodes_in_tree[len(nodes_in_tree) - 1].neighbour_indexes.append(i)

#Adding Goal to Tree
nodes_in_tree.append(Nodes(goal[0], goal[1], [], 0, 0, len(nodes_in_tree)))

for i in range(len(nodes_in_tree) - 1):
    dist = math.sqrt((nodes_in_tree[i].x_coord - goal[0]) ** 2 + (nodes_in_tree[i].y_coord - goal[1]) ** 2)

    if(dist <= fixed_distance and CheckIfValidConnection(goal, nodes_in_tree[i])):
        nodes_in_tree[i].neighbour_indexes.append(len(nodes_in_tree) - 1)
        nodes_in_tree[len(nodes_in_tree) - 1].neighbour_indexes.append(i)

print("Tree plotted.")
#Astar Algorithm to find goal
#Initialise costs and arrange array
ranking_of_nodes = []
for node in nodes_in_tree:
    node.cost = 9999999
nodes_in_tree[0].cost = 0
for index in nodes_in_tree[0].neighbour_indexes:
    nodes_in_tree[index].cost = math.sqrt((nodes_in_tree[index].x_coord - nodes_in_tree[0].x_coord) ** 2 + (nodes_in_tree[index].y_coord - nodes_in_tree[0].y_coord) ** 2) + math.sqrt((nodes_in_tree[index].x_coord - goal[0]) ** 2 + (nodes_in_tree[index].y_coord - goal[1]) ** 2)
    nodes_in_tree[index].parent_index = 0

for node in nodes_in_tree:
    ranking_of_nodes.append([node.self_index, node.cost])
ranking_of_nodes.sort(key = lambda x: x[1])

goal_reached = False
while(len(ranking_of_nodes) > 0 and (not goal_reached)):
    if(ranking_of_nodes[0][0] == len(nodes_in_tree) - 1):
            goal_reached = True

    for index in nodes_in_tree[ranking_of_nodes[0][0]].neighbour_indexes:
        new_cost = nodes_in_tree[ranking_of_nodes[0][0]].cost + math.sqrt((nodes_in_tree[index].x_coord - nodes_in_tree[ranking_of_nodes[0][0]].x_coord) ** 2 + (nodes_in_tree[index].y_coord - nodes_in_tree[ranking_of_nodes[0][0]].y_coord) ** 2) + math.sqrt((nodes_in_tree[index].x_coord - goal[0]) ** 2 + (nodes_in_tree[index].y_coord - goal[1]) ** 2)
        if(new_cost < nodes_in_tree[index].cost):
            nodes_in_tree[index].cost = new_cost
            nodes_in_tree[index].parent_index = ranking_of_nodes[0][0]
    
    ranking_of_nodes.pop(0)
    for ranking in ranking_of_nodes:
        ranking[1] = nodes_in_tree[ranking[0]].cost
    ranking_of_nodes.sort(key = lambda x: x[1])

print(len(ranking_of_nodes))
#Visualizing graph
#Drawing nodes
counter = 0
for node in nodes_in_tree:
    counter += 1
    print(counter)
    for neighbour_index in node.neighbour_indexes:
        #circle = plt.Circle((node.x_cood, node.y_coord), fixed_distance, color = 'b', fill = False)
        #ax.add_patch(circle)
        plt.plot([node.x_coord, nodes_in_tree[neighbour_index].x_coord], [node.y_coord, nodes_in_tree[neighbour_index].y_coord], "r.-", markersize = 3, linewidth = 0.3)


#draw obstacles
#for obj in obstacles:
#    xs, ys = zip(*obj.coord_list)
#    plt.plot(xs, ys, "c-")
#Connecting Goal to the start point
curr_index = len(nodes_in_tree) - 1
while(curr_index != 0):
    parent_index = nodes_in_tree[curr_index].parent_index
    plt.plot([nodes_in_tree[curr_index].x_coord, nodes_in_tree[parent_index].x_coord], [nodes_in_tree[curr_index].y_coord, nodes_in_tree[parent_index].y_coord], 'b.-', markersize = 5, linewidth = 0.5)
    curr_index = parent_index
#draw goal
#xs, ys = zip(*goal.coord_list)
#plt.plot(xs, ys)
plt.show()
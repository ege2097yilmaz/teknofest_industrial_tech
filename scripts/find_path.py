#!/usr/bin/env python3

from __future__ import print_function

from typing import Dict, List

import rospy
from teknofest_industrial_tech.srv import directions, directionsResponse

# map values
graph = {}

graph[1] = {2,21}
graph[2] = {1,3,12}
graph[3] = {2,4,13}
graph[4] = {3,5,24}
graph[5] = {4,6,15}
graph[6] = {5,7,16}
graph[7] = {6,27}
graph[12] = {2,22}
graph[13] = {3,23}
graph[15] = {5,25}
graph[16] = {6,26}
graph[21] = {1,22}
graph[22] = {12,21,23}
graph[23] = {13,22,24}
graph[24] = {4,23,25}
graph[25] = {15,24,26}
graph[26] = {16,25,27}
graph[27] = {7,26}

positions = {
    "S1": 4,
    "S2": 24,
    "A": 12,
    "B": 13,
    "C": 15,
    "D": 16,
    "I": 7,
    "F": 27,
    "G": 21,
    "E": 1
}

# depth first search
def shortest_path(graph, node1, node2):
    path_list = [[node1]]
    path_index = 0

    previous_nodes = {node1}

    if node1 == node2:
        return path_list[0]
    
    while path_index < len(path_list):
        current_path = path_list[path_index]
        last_node = current_path[-1]
        next_nodes = graph[last_node]

        if node2 in next_nodes:
            current_path.append(node2)
            return current_path
        
        for next_node in next_nodes:
            if not next_node in previous_nodes:
                new_path = current_path[:]
                new_path.append(next_node)
                path_list.append(new_path)
                previous_nodes.add(next_node)
        path_index += 1
    return []

# defining directions correspond to shortest path and append into direction_list
def find_directions(path):
    subtrahend = 0
    minuend = 1

    direction_list = []
    
    while minuend < len(path):
        difference = path[minuend] - path[subtrahend]

        if difference >= 1 and difference < 10:
            direction_list.append("R")
        
        elif difference > -10 and difference < 1:
            direction_list.append("L")
        
        elif difference >= 10:
            direction_list.append("D")
        
        elif difference <= -10:
            direction_list.append("U")
        subtrahend += 1
        minuend += 1
    return direction_list

# return goal lists
def get_goal(req):
    go_toload = find_directions(shortest_path(graph, positions["S1"], positions[req.load]))
    go_tounload = find_directions(shortest_path(graph, positions[req.load], positions[req.unload]))
    go_start = find_directions(shortest_path(graph, positions[req.unload], positions["S1"]))

    return directionsResponse(go_toload, go_tounload, go_start)

def directions_publisher():
    rospy.init_node('path_planning')
    s = rospy.Service('find_path', directions, get_goal)
    rospy.spin()

if __name__ == "__main__":
    directions_publisher()
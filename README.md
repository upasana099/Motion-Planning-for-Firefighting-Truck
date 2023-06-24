# Autonomous Fire Truck Path Planning

## Project Description

Autonomous vehicles are becoming increasingly common on roads worldwide, and as they become more prevalent, they must be capable of performing complex maneuvers in tight and cluttered spaces, such as parking lots. This project focuses on developing advanced path planning algorithms for autonomous fire trucks to efficiently navigate and extinguish fires in a simulated environment.

### Environment 
The simulated environment comprises a 250-meter flat square field filled with various obstacles, including large patches of thick brush, trees, and weeds that resemble giant tetrominoes. Within this field, a firetruck is deployed to put out fires that may arise. The goal is to find optimal paths for the firetruck to quickly and safely reach and extinguish the fires.


![WhatsApp Image 2023-06-24 at 03 25 50](https://github.com/upasana099/Motion-Planning-for-Firefighting-Truck/assets/89516193/3c92bdf6-bafd-4498-9792-a411a2da7d1e)



## Pre-Requisites

- PRM (Probabilistic Roadmap) is a probabilistic algorithm used for path planning in high-dimensional spaces. It efficiently explores the environment, but it does not guarantee finding an optimal path in all cases.
- A* (A-star) is an efficient pathfinding algorithm known for finding the shortest path in a graph by utilizing both actual and estimated costs.
- Bresenham's algorithm is used to calculate straight paths between two points in a grid-based environment.

## Pseudo Code

### PRM:

```python
initialize roadmap R with an empty set of vertices and edges

while R has fewer than n vertices do:
    randomly generate a new configuration q
    if q is collision-free then add q to R

for each pair of vertices u,v in R do:
    if the straight-line path between u and v is collision-free then add an edge (u,v) to R

return R

```
### PRM:

```python
function A*(start, goal, cost)
    initialize the open list and closed list as empty sets
    add the start node to the open list with a cost of 0

    while the open list is not empty do:
        select the node with the lowest cost from the open list, call it current
        if current is the goal node then:
            return the path from start to goal

        for each neighbor of current do:
            calculate the cost of moving from current to the neighbor
            if the neighbor is not in the open list and not in the closed list then:
                set the neighbor's parent to current
                set the neighbor's heuristic cost (h) and total cost (f) 
                add the neighbor to the open list
            else if the neighbor is already in the open list or closed list then:
                if the new cost to get to the neighbor is lower than the old cost then:
                    update the neighbor's parent, h cost, and f cost

        add current to the closed list

    return failure
```
## Implementation
The project consists of the following files:

* algorithms.py: Contains the implementation of the PRM and A* path planning algorithms.
* wumpus.py: Simulates a Wumpus starting a fire using the A* algorithm in a 2D grid.
* world.py: Simulates a fire truck extinguishing the fire using PRM as the global planner and A* as the local planner.

### Results

*Wumpus agent animation



https://github.com/upasana099/Motion-Planning-for-Firefighting-Truck/assets/89516193/7d81e264-1a9a-4c1d-ba06-58fa3e5f03bb


* Fire truck agen animation

 

https://github.com/upasana099/Motion-Planning-for-Firefighting-Truck/assets/89516193/f9244646-7af1-464d-91c6-894cff591b96




### Performance Evaluation
* Fire Extinguished Rate: 96.477%
* Fire Extinguished to Fire Burned Ratio: 5.907503
* Computational Evaluation
* A* Execution Time: 184.65 seconds
* PRM Execution Time: 335.22 seconds

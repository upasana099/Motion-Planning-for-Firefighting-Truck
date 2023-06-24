from cmath import sqrt
import pygame
import numpy as np
from random import choice, uniform
from random import randint
import random
import math
import heapq
from sklearn.neighbors import KDTree
from heapq import heappop, heappush
import matplotlib as plt
import networkx as nx
from os import path
from numpy import pi, sqrt

TURQUOISE = (64, 224, 208)

#parameters for displaying output
obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
car1_posx = 30
car1_posy = 10
car2_posx = 130
car2_posy = 10
police_carx = 0
police_cary = 180
agent_theta = 0
car_width = 30
car_height = 20 
padding = 5

# start and end for motion planning
police_start = [police_carx + 1+5, police_cary + car_height/2,0]
police_goal = [car1_posx+car_width+15+1+5,10 + car_height/2,0]

agent_boundary = [[-1,29,29,-1],[-10,-10,10,10],[1,1,1,1]]

# Define constants for grid size and tetromino shapes
GRID_SIZE = (80, 80)
TETROMINOES = [
    [(0, 0), (0, 1), (0, 2), (0, 3)],  # I shape
    [(0, 0), (0, 1), (0, 2), (1, 2)],  # L shape
    [(0, 0), (0, 1), (0, 2), (-1, 2)],  # L2 shape
    [(0, 0), (0, 1), (0, 2), (1, 1)],  # T shape
    [(0, 0), (0, 1), (-1, 1), (1, 0)],  # Z shape
    [(0, 0), (0, 1), (1, 1), (-1, 0)],  # Z2 shape
    [(0, 0), (0, 1), (0, 2), (1, 0)]  # S shape
]

# Define function to create obstacles
def create_obstacles(surface, cell_size):
    num_obstacles = int(GRID_SIZE[0] * GRID_SIZE[1] * 0.027) # 10 percent occupancy
    obstacles = []
    for i in range(num_obstacles):
        tetromino = random.choice(TETROMINOES)
        x = random.choice(range(5,GRID_SIZE[0]-5))
        y = random.choice(range(5,GRID_SIZE[1] - len(tetromino)))
        obstacle_cells = []
        rect_list = []
        for j, (dx, dy) in enumerate(tetromino):
            cell_x = (x + dx) * cell_size
            cell_y = (y + dy) * cell_size
            rect = pygame.Rect(cell_x-cell_size/2, cell_y - cell_size/2, cell_size, cell_size)
            pygame.draw.rect(surface, 'RED', rect)
            obstacle_cells.append([cell_x + cell_size//2, cell_y + cell_size//2])
            rect_list.append(rect)
        obstacles.append(obstacle_cells)

    return obstacles, rect_list


# Initialize Pygame
pygame.init()
# Set up screen and clock
screen_size = (400, 400)
screen = pygame.display.set_mode(screen_size)
# Fill screen with white
screen.fill('WHITE')
clock = pygame.time.Clock()

# Set up grid and cell size
cell_size = min(screen_size[0] // GRID_SIZE[0], screen_size[1] // GRID_SIZE[1])

# Create obstacles
create_obstacles(screen, cell_size)


# Define function to calculate distance between two points
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


# Define function to create PRM roadmap
def prm(num_nodes, cell_size):
    # Create empty graph
    graph = {}
    # Create obstacles
    obstacles,rect_obstacles = create_obstacles(screen, cell_size)
    #print("sgfasgsdgdsgdfsgdfsgdfsg",obstacles)
    # Create random nodes
    nodes = []
    nodes_generated = 0
    while nodes_generated < num_nodes:
        collided = False
        node = [random.randint(0, GRID_SIZE[0]*cell_size), random.randint(0, GRID_SIZE[1]*cell_size)]
        rect_node = pygame.Rect(node[0],node[1],cell_size,cell_size)
        for rect_obstacle in rect_obstacles:
            if rect_node.colliderect(rect_obstacle):
                collided = True
                break
        if collided:
            continue
        else:
            nodes.append(node)
            nodes_generated +=1
    #nodes = [(random.randint(0, GRID_SIZE[0]*cell_size), random.randint(0, GRID_SIZE[1]*cell_size)) for i in range(num_nodes)]
    # Add nodes to graph
    for i, node in enumerate(nodes):
        graph[tuple(node)] = []
        # Find nearest neighbors
        for j, other_node in enumerate(nodes):
            if i != j:
                # Check if edge is collision-free
                edge_valid = True
               # print(j,len(obstacles))
                for obstacle in obstacles:
                    for k in range(len(obstacle)-1):
                        if distance(node, other_node) < distance(obstacle[k], obstacle[k+1]):
                            edge_valid = False
                            break  
                if edge_valid:
                    graph[tuple(node)].append(other_node)

    return graph, nodes


# Define function to visualize PRM graph
def visualize_prm(graph, nodes, cell_size):
    # Initialize Pygame
    pygame.init()
    # Set window size
    screen_size = (GRID_SIZE[0]*cell_size, GRID_SIZE[1]*cell_size)
    # Create screen
    screen = pygame.display.set_mode(screen_size)
    # Set window title
    pygame.display.set_caption('PRM Graph')
    # Draw nodes
    for node in nodes:
        pygame.draw.circle(screen, 'WHITE', node, cell_size//2)
    # Draw edges
    for i, neighbors in graph.items():
        for j in neighbors:
           pygame.draw.line(screen, 'WHITE', nodes[i], nodes[j])
           
    # Draw obstacles
    obstacles,rect_obstacles = create_obstacles(screen, cell_size)
    for obstacle in obstacles:
        print(obstacle)
        #pygame.draw.circle(screen, 'BLUE', obstacle)
    # Update screen
    pygame.display.flip()
    # Wait for user to close window
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

obstacle_posx = 70
obstacle_posy = 90
obstacle_width = 50
obstacle_height = 50
car1_posx = 30
car1_posy = 10
car2_posx = 130
car2_posy = 10
police_carx = 0
police_cary = 180
agent_theta = 0
car_width = 30
car_height = 20 
padding = 5

wheelbase = 28
steering_angle = 30
vel = 1

agent_bound = [[police_carx,police_cary,1],[police_carx+car_width,police_cary,1],[police_carx+car_width,police_cary+car_height,1],[police_carx,police_cary+car_height,1]]
obstacle = [[obstacle_posx-padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy-padding],[obstacle_posx+obstacle_width+padding,obstacle_posy+obstacle_height+padding],[obstacle_posx-padding,obstacle_posy+obstacle_height+padding]]
car1 = [[car1_posx-padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy-padding],[car1_posx+car_width+padding,car1_posy+car_height+padding],[car1_posx-padding,car1_posy+car_height+padding]]
car2 = [[car2_posx-padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy-padding],[car2_posx+car_width+padding,car2_posy+car_height+padding],[car2_posx-padding,car2_posy+car_height+padding]]

def collision_check(a, b):
    polygons = [a, b]

    for polygon in polygons:
        for i, p1 in enumerate(polygon):
            p2 = polygon[(i + 1) % len(polygon)]
            normal = (p2[1] - p1[1], p1[0] - p2[0])
            minA, maxA = None, None
            for p in a:
                projected = normal[0] * p[0] + normal[1] * p[1]
                if minA is None or projected < minA:
                    minA = projected
                if maxA is None or projected > maxA:
                    maxA = projected
            minB, maxB = None, None
            for p in b:
                projected = normal[0] * p[0] + normal[1] * p[1]
                if minB is None or projected < minB:
                    minB = projected
                if maxB is None or projected > maxB:
                    maxB = projected
            if maxA < minB or maxB < minA:
                return False

    return True
def get_boundary(x,y,theta):
    tx = x 
    ty = y 
    th = theta-police_start[2]
    homogeneous_matrix = [[math.cos(th*(pi/180)),-math.sin(th*(pi/180)),tx],[math.sin(th*(pi/180)),math.cos(th*(pi/180)),ty]]
    mat_mul = np.dot(homogeneous_matrix,agent_boundary)
    new_boundary = [[mat_mul[0][0],mat_mul[1][0]],[mat_mul[0][1],mat_mul[1][1]],[mat_mul[0][2],mat_mul[1][2]],[mat_mul[0][3],mat_mul[1][3]]]
    return new_boundary


def valid_point(x, y, theta):
     # Get the boundary of the car at the given position and angle
    boundary = get_boundary(x, y, theta)
    bounds = (1, car_height, 200 - car_width, 200 - car_height / 2.0)
    
    if any(coord < bound for coord, bound in zip((x, y), bounds)) or collision_check(boundary, obstacle) or any(collision_check(boundary, car) for car in (car1, car2)):
        return False
    
    return True

def get_neighbours(x,y,theta):
    neighbour = []
    for i in range(-steering_angle,steering_angle+1,5):
        x_dot = vel*math.cos(theta*(pi/180))
        y_dot = vel*math.sin(theta*(pi/180))
        theta_dot = (vel*math.tan(i*(pi/180))/wheelbase)*(180/pi)
        if(valid_point(x+x_dot,y+y_dot,theta+theta_dot)): # to check if the neighbour position is a valid one before adding it to the list of neighbour
            neighbour.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,1,i])
        if(valid_point(x-x_dot,y-y_dot,theta-theta_dot)): # to check if the neighbour position is a valid one before adding it to the list of neighbour
            neighbour.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2)+360)%360,-1,i])
    return neighbour

def straight_available(x,y):
    boundary_line = [[x,y],[police_goal[0],police_goal[1]],[police_goal[0]+1,police_goal[1]],[x+1,y]]
    if collision_check(boundary_line,obstacle):
        return False
    if collision_check(boundary_line,car1):
        return False
    return True

def cost_function(x1,y1,x2,y2):
    distance = math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    return distance

def priority(queue): 
    min = math.inf
    index = 0
    for check in range(len(queue)):
        _,value,_,_ = queue[check]
        if value<min:
            min = value
            index = check 
    return index


def hurestic_function(x,y):
    theta_ = 0
    theta = 0
    distance = sqrt((pow(police_goal[0]-x,2)+pow(police_goal[1]-y,2))) 
    distance += sqrt(((pow((police_goal[0]+car_width)-(x+car_width*math.cos(theta*(pi/180))),2)+pow((police_goal[1]+car_height)-(y+car_width*math.sin(theta*(pi/180))),2)))) # distance of the front axle
    if straight_available(x,y) and not(x>police_goal[0]-5 and y>police_goal[1]-5 and x <police_goal[0]+5 and y <police_goal[1]+5): 
        theta_ = abs((360 + (math.atan2(y-police_goal[1],x-police_goal[0]))*(180/pi))%360 - theta+180) 
    hurestic = distance+theta_
    return hurestic

def check_visited(current,visited):
    for x,y,th in visited:
        if current[0]== x and current[1]== y and current[2]==th :
            return True
    return False

def prm_A_star(start,police_goal,graph):
    open_set = []
    visited = []
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    while len(open_set)>0:
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] 
        open_set.pop(index)
        if not check_visited([round(shortest[0]),round(shortest[1])],visited): # Check if node already visited
            visited.append([round(shortest[0]),round(shortest[1])])
            if round(shortest[0]) <= police_goal[0]+5 and round(shortest[0]) >= police_goal[0]-5 and round(shortest[1]) <= police_goal[1]+5 and round(shortest[1]) >= police_goal[1]-5: #goal condition
                return path
            print(shortest)
            neighbours= graph[shortest[0],shortest[1]]
            for neighbour in neighbours:
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                temp_tcost = temp_gcost+(0.9*hurestic_function(neighbour[0],neighbour[1]))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path

   

def A_star(start,police_goal):
    open_set = []
    visited = []
    tcost = 0
    gcost = 0
    path = [start]
    open_set.append((start,tcost,gcost,path))
    while len(open_set)>0:
        index = priority(open_set)
        (shortest,_,gvalue,path) = open_set[index] 
        open_set.pop(index)
        if not (check_visited([round(shortest[0]),round(shortest[1])],visited)): # Check if node already visited
            visited.append([round(shortest[0]),round(shortest[1])])
            if round(shortest[0]) <= police_goal[0]+5 and round(shortest[0]) >= police_goal[0]-5 and round(shortest[1]) <= police_goal[1]+5 and round(shortest[1]) >= police_goal[1]-5: #goal condition
                return path
            neighbours= get_neighbours(shortest[0],shortest[1]) 
            for neighbour in neighbours:
                temp_gcost = gvalue+(0.1*cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                temp_tcost = temp_gcost+(0.9*hurestic_function(neighbour[0],neighbour[1]))
                open_set.append((neighbour,temp_tcost,temp_gcost,path+ [neighbour]))
    print("not working")      
    return path

pygame.init()
# Set window size
screen_size = (GRID_SIZE[0]*cell_size, GRID_SIZE[1]*cell_size)
# Create screen
screen = pygame.display.set_mode(screen_size)

num_nodes = 100
cell_size = 10

graph, nodes = prm(num_nodes, cell_size)
print(graph)

prm_path = prm_A_star(nodes[0], nodes[10],graph)
path = []
for i in range(len(prm_path)-1):
    path += A_star(i,i+1)

for i in range(len(path)-1):
    pygame.draw.line(screen,(255,255,255),[path[i,0],path[i,1]],[path[i+1,0],path[i+1,1]])
    
    pygame.display.update()
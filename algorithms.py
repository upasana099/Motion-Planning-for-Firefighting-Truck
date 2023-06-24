from global_var import *
from fire_truck import *
import heapq
import numpy as np
from scipy.spatial import cKDTree
import global_var
import fire_truck
import math

def calc_dist(u, v):
    dist = ((u[0] - v[0])**2 + (u[1] - v[1])**2)**0.5
    return dist

def AStar(self, robot, start, goal):
    q = []
    global_var.DISTANCE[robot.position[0]][robot.position[1]] = 0
    values = robot.position[0], robot.position[1], robot.heading
    heapq.heappush(q, (global_var.DISTANCE[robot.position[0]][robot.position[1]] + calc_dist(start, goal), values))
    global_var.PREVIOUS[position[0]][position[1]] = None
    delta_t = 0.5
    us_values = [-robot.max_velocity, robot.max_velocity]
    max_steer = math.atan2(robot.wheelbase, robot.turning_radius)
    u_phi_values = [-max_steer, 0, max_steer]
    while (len(q) != 0):
        priority, u = heapq.heappop(q)
        x = u[0]
        y = u[1]
        current_heading = u[2]
        possible_states = []
        for us in (us_values):
            for u_phi in (u_phi_values):
                theta_next = us * (math.tan(u_phi) / robot.wheelbase) * delta_t + current_heading
                x_next = us * math.cos(theta_next) * delta_t + x
                y_next = us * math.sin(theta_next) * delta_t + y
                possible_position = round(x_next), round(y_next)
                heading = theta_next
                if robot.check_possible_config(self.grid, possible_position, heading):
                    values = possible_position[0], possible_position[1], heading
                    if values[0] != u[0] and values[1] != u[1]:
                        possible_states.append(values)
        for v in possible_states:
            if ((DISTANCE[v[0]][v[1]] > DISTANCE[x][y] + calc_dist(u, v))):
                dist = calc_dist(u, v)
                DISTANCE[v[0]][v[1]] = DISTANCE[x][y] + calc_dist(u, v)
                reverse = 0
                if ((v[0] - u[0]) / ((math.cos(v[2])) * delta_t) < 0):
                    reverse = math.pi * robot.turning_radius
                global_var.PREVIOUS[v[0]][v[1]] = u
                heapq.heappush(q, (DISTANCE[v[0]][v[1]] + (calc_dist(v, goal)) + 2*reverse, v))
                
        if (calc_dist(u, goal) <= 10):
            # print("A-Star Search complete. Tracing path....")
            trace_path(self, robot, start, u)
            return (True)
        if (len(q) == 0):
            # print("ERROR - A-Star: The given obstacle field does not have a path to destination.")
            return (False)
    return True

def trace_path(self, robot, start, end):
    result = []
    while (end != start):
        result.append(end)
        end = global_var.PREVIOUS[end[0]][end[1]]
    result.append(start)
    result.reverse()
    for i in result:
        robot.clear_robot(self.grid)
        robot.position = i[0], i[1]
        robot.heading = i[2]
        robot.make_robot()
        robot.draw_robot(self.grid)
        self.extinguish(robot)
        self.draw()
    # print("Done.")


def bresenham_line(edge):
    x, y = edge[0]
    x2, y2 = edge[1]
    line = []
    line.append(edge[0])
    dx = abs(x2 - x)
    dy = -abs(y2 - y)
    sx = 1 if x < x2 else -1
    sy = 1 if y < y2 else -1
    err = dx + dy
    while (x != x2 or y != y2):
        e2 = 2*err
        if (e2 >= dy):
            err += dy
            x += sx
        if (e2 <= dx):
            err += dx
            y+=sy
        temp = x, y
        line.append(temp)
    line.append(edge[1])
    return line

def is_collision_free(grid, edge):
    rasterized_line = bresenham_line(edge)
    for point in rasterized_line:
        if (grid[point[0]][point[1]].color == GREEN or grid[point[0]][point[1]].color == RED or grid[point[0]][point[1]].color == BLACK):
            return False
    return True

def trace_global_path(world, robot, start_index, goal_index):
    result = []
    while (goal_index != start_index):
        result.append(goal_index)
        goal_index = PREVIOUS[world.VERTICES[goal_index][0]][world.VERTICES[goal_index][1]]
    result.append(start_index)
    result.reverse()
    for i in range(len(result) - 1):
        world.reinitialize()
        start_local = robot.position[0], robot.position[1], robot.heading
        goal_local = world.VERTICES[result[i + 1]][0], world.VERTICES[result[i + 1]][1]
        AStar(world, robot, start_local, goal_local)
    print("Done.!!")

def reset_prm(world):
    ##for goal point
    for i in world.EDGES[len(world.EDGES) - 1]:
        world.EDGES[i].pop(len(world.EDGES[i]) - 1)
    world.EDGES.pop(len(world.EDGES) - 1)
    world.VERTICES.pop(len(world.VERTICES) - 1)
    ##For start point
    world.EDGES.pop(len(world.EDGES) - 1)
    world.VERTICES.pop(len(world.VERTICES) - 1)

def PRM(world, robot, start, goal):
    n = 100
    while(len(world.VERTICES) < n):
        i = random.randint(0, world.ROWS - 1)
        j = random.randint(0, world.ROWS - 1)
        while (world.CONFIGURATION_SPACE[i][j] == False):
            i = random.randint(0, world.ROWS - 1)
            j = random.randint(0, world.ROWS - 1)
        point = i, j
        if point not in world.VERTICES:
            world.VERTICES.append(point)

    tree = cKDTree(world.VERTICES)
    for i in range(len(world.VERTICES)):
        world.NNlist.append([])
        world.EDGES.append([])
        dist, ind = tree.query([world.VERTICES[i]], k=100)
        for j in ind[0][1:]:
            world.NNlist[i].append(j)
            edge = world.VERTICES[i], world.VERTICES[j]
            if (is_collision_free(world.grid, edge)):
                world.EDGES[i].append(j)

    start_position = start[0], start[1]
    goal_position = goal[0], goal[1]

    world.VERTICES.append(start_position) ##  Start vertex in list of vertices
    last = len(world.VERTICES) - 1
    dist, ind = tree.query([start_position], k = 6) 
    world.NNlist.append([]) ## Nearest neighbours for start
    world.EDGES.append([]) ## Edges for start
    for j in ind[0][1:]:
        world.NNlist[last].append(j)
        edge = world.VERTICES[last], world.VERTICES[j]
        if (is_collision_free(world.grid, edge)):
            world.EDGES[last].append(j)


    world.VERTICES.append(goal_position) ##  Start vertex in list of vertices
    last = len(world.VERTICES) - 1
    dist, ind = tree.query([goal_position], k = 6) 
    world.NNlist.append([]) ## Nearest neighbours for start
    world.EDGES.append([]) ## Edges for start
    for j in ind[0][1:]:
        world.NNlist[last].append(j)
        edge = world.VERTICES[last], world.VERTICES[j]
        world.EDGES[last].append(j)
        world.EDGES[j].append(last)

    q = []
    global_var.DISTANCE[start_position[0]][start_position[1]] = 0
    dist = calc_dist(start_position, goal_position)
    heapq.heappush(q, (global_var.DISTANCE[start_position[0]][start_position[1]] + dist, len(world.VERTICES) - 2))
    while (len(q) != 0):
        priority, u = heapq.heappop(q)
        node_u = world.VERTICES[u]
        if (u != len(world.VERTICES) - 1):
            for v in world.EDGES[u]:
                node_v = world.VERTICES[v]
                if (global_var.DISTANCE[node_v[0]][node_v[1]] > global_var.DISTANCE[node_u[0]][node_u[1]] + calc_dist(node_u, node_v)):
                    dist = calc_dist(node_u, node_v)
                    global_var.DISTANCE[node_v[0]][node_v[1]] = global_var.DISTANCE[node_u[0]][node_u[1]] + dist
                    heapq.heappush(q, (global_var.DISTANCE[node_v[0]][node_v[1]] + calc_dist(node_v, goal_position), v))
                    PREVIOUS[node_v[0]][node_v[1]] = u
        if (u == len(world.VERTICES) - 1):
            # print("PRM Search complete. Tracing path....")
            trace_global_path(world, robot, len(world.VERTICES) - 2, len(world.VERTICES) - 1)
            reset_prm(world)
            return (True)
        if (len(q) == 0):
            # print("ERROR - PRM: The given obstacle field does not have a path to destination.")
            return (False)
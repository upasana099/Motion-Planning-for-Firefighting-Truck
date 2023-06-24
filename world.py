import pygame
import math
import random
from global_var import *
from robot import *
import threading
import random
from collections import deque
import algorithms
import time


class Block:
    def __init__(self, row, col, color, size, type):
        self.row = row
        self.col = col
        self.size = size
        self.x = row * size
        self.y = col * size
        self.color = color
        self.type = type

    def draw_block(self, world):
        pygame.draw.rect(world, self.color, (self.x, self.y, self.size, self.size))

class World:
    def __init__(self, SIZE, ROWS, CONFIGURATION_SPACE, VERTICES, EDGES, OBSTACLE_ROWS, PATCH_DENSITY, FOREST_DENSITY):
        self.world = []
        self.SIZE = SIZE
        self.ROWS = ROWS
        self.CONFIGURATION_SPACE = CONFIGURATION_SPACE
        self.VERTICES = VERTICES
        self.EDGES = EDGES
        self.OBSTACLE_ROWS = OBSTACLE_ROWS
        self.PATCH_DENSITY = PATCH_DENSITY
        self.FOREST_DENSITY = FOREST_DENSITY
        self.block_size = self.SIZE // self.ROWS
        self.grid = []          ## reinitialize to all green
        self.obstacle = []
        self.patch_centers = []
        self.patch_status = []      ##initialize to all False
        self.patch_obstacle_coordinates = []
        self.patch_obstacle_status = []     ##initialize to all False
        self.fire_spread_dist = 30
        self.time = 0
        self.AStar_time = 0
        self.PRM_time = 0
        self.total_count = 0 
        self.burned_count = 0               ##initialize to 0
        self.intact_count = 0               ##initialize to self.total_count
        self.extinguished_count = 0          ##initialize to 0
        self.ignite_patch_center = 0          ##initialize to 0
        self.ignite_patch_obstacle = 0          ##initialize to 0
        self.queue = deque()                    ##initilaize to 0
        self.NNlist = []

    def create_world(self):
        return pygame.display.set_mode((self.SIZE, self.SIZE))

    def reset_world(self):
        for row in self.grid:
            for element in row:
                if (element.color == BLACK or element.color == RED):
                    element.color = GREEN
        for status in self.patch_status:
            status = False
        for patch in self.patch_obstacle_status:
            for obstacle_status in patch:
                if obstacle_status == True:
                    obstacle_status = False
        self.burned_count = 0
        self.intact_count = self.total_count
        self.extinguished_count = 0
        self.ignite_patch_center = 0
        self.ignite_patch_obstacle = 0
        self.queue.clear()
        self.queue = deque()
    def reinitialize(self):
        for i in range(self.ROWS):
            for j in range(self.ROWS):
                DISTANCE[i][j] = (math.inf)
                PREVIOUS[i][j] = []
    def draw_grid_borders(self):
        for i in range(self.ROWS):
            pygame.draw.line(self.world, BLACK, (0, i * self.block_size), (self.SIZE, i * self.block_size))
            for j in range(self.ROWS):
                pygame.draw.line(self.world, BLACK, (j * self.block_size, 0), (j * self.block_size, self.SIZE))

    def draw(self):
        for row in self.grid:
            for block in row:
                block.draw_block(self.world)
        # self.draw_grid_borders()
        pygame.display.update()


    def initialize_world(self):
        self.world = self.create_world()
        for i in range(self.ROWS):
            self.grid.append([])
            DISTANCE.append([])
            PREVIOUS.append([])
            self.CONFIGURATION_SPACE.append([])
            for j in range(self.ROWS):
                block = Block(i, j, WHITE, self.block_size, None)
                self.grid[i].append(block)
                DISTANCE[i].append(math.inf)
                PREVIOUS[i].append([])
                self.CONFIGURATION_SPACE[i].append(True)


  
    def make_random_obstacle(self):
        self.obstacle.clear()
        tetris = ['I', 'r', 'N', 'T']
        value = random.choice(tetris)
        if (value == 'I'):
            for i in range(1):
                self.obstacle.append([])
                for j in range(4):
                    block = Block(i, j, GREEN, self.block_size, type="I")
                    self.obstacle[i].append(block)

        elif (value == 'r'):
            for i in range(2):
                self.obstacle.append([])
                for j in range(3):
                    block = Block(i, j, WHITE, self.block_size, None)
                    if (i == 0 and j > 0):
                        block.color = WHITE
                    else:
                        block.color = GREEN
                        block.type = "r"
                    self.obstacle[i].append(block)

        elif (value == 'N'):
            for i in range(2):
                self.obstacle.append([])
                for j in range(3):
                    block = Block(i, j, WHITE, self.block_size, None)
                    if ((i == 1 and j == 0) or (i == 0 and j == 2)):
                        block.color = WHITE
                    else:
                        block.color = GREEN
                        block.type = "N"
                    self.obstacle[i].append(block)

        elif (value == 'T'):
            for i in range(2):
                self.obstacle.append([])
                for j in range(3):
                    block = Block(i, j, WHITE, self.block_size, None)
                    if (i == 0 and j != 1):
                        block.color = WHITE
                    else:
                        block.color = GREEN
                        block.type = "T"
                    self.obstacle[i].append(block)

    def check_overlap(self, random_row, random_col):
        for i in range(len(self.obstacle)):
            for j in range(len(self.obstacle[i])):
                if (self.obstacle[i][j].color == GREEN and self.grid[random_row + i][random_col + j].color == GREEN):
                    return True
        return False

    def place_random_obstacle(self, x, y):
        cells = self.OBSTACLE_ROWS ** 2
        obs_cells = self.PATCH_DENSITY * cells
        count = 0
        obstacle_data = []
        obstacle_status = []
        top_corner = round((self.OBSTACLE_ROWS - 1)/2)
        for i in range(-top_corner, top_corner):
            for j in range(-top_corner, top_corner):
                self.CONFIGURATION_SPACE[i + x][j + y] = False
        while (count < obs_cells - 4):
            condition = True
            while condition:
                self.make_random_obstacle()
                max_row = round((self.OBSTACLE_ROWS - 1)/2) - len(self.obstacle)
                max_col = round((self.OBSTACLE_ROWS - 1)/2) - len(self.obstacle[0])
                random_row = random.randint(-max_row - len(self.obstacle), max_row)
                random_col = random.randint(-max_col - len(self.obstacle[0]), max_col)
                if (self.check_overlap(random_row + x, random_col + y) == False):
                    self.intact_count += 1
                    self.total_count += 1
                    data = []
                    condition = False
                    for i in range(len(self.obstacle)):
                        for j in range(len(self.obstacle[i])):
                            self.grid[random_row + i + x][random_col + j + y].color = self.obstacle[i][j].color
                            # self.CONFIGURATION_SPACE[random_row + i + x][random_col + j + y] = False
                            if (self.obstacle[i][j].color == GREEN):
                                coordinate = random_row + i + x, random_col + j + y
                                data.append(coordinate)
                            count +=1
                    obstacle_data.append(data)
                    obstacle_status.append(False)
        return obstacle_data, obstacle_status

    def draw_world(self):
        self.initialize_world()
        total_possible_patches = (self.ROWS // self.OBSTACLE_ROWS) ** 2
        total_forest_patches = round(self.FOREST_DENSITY * total_possible_patches)
        patch_locations = []
        x = round((self.OBSTACLE_ROWS + 1) / 2)
        y = round((self.OBSTACLE_ROWS + 1) / 2)
        for i in range(round(math.sqrt(total_possible_patches))):
            patch_locations.append([])
            for j in range(round(math.sqrt(total_possible_patches))):
                value = x, y, 0
                patch_locations[i].append(value)
                y = y + self.OBSTACLE_ROWS
            x = x + self.OBSTACLE_ROWS
            y = round((self.OBSTACLE_ROWS + 1) / 2)
        count = 0
        while (count < total_forest_patches):
            i = random.randint(0, len(patch_locations) - 1)
            j = random.randint(0, len(patch_locations) - 1)
            value = patch_locations[i][j]
            status = value[2]
            if (status == 0):
                x = patch_locations[i][j][0]
                y = patch_locations[i][j][1]
                obstacle_data, obstacle_status = self.place_random_obstacle(x, y)
                
                patch_locations[i][j] = x, y, 1
                patch_center = x, y
                self.patch_centers.append(patch_center) #list of centers of all patches
                self.patch_status.append(False) #list of status of fire at any point in a particular patch
                self.patch_obstacle_coordinates.append(obstacle_data) #list of coordinates of all obstacles in a patch for all patches
                self.patch_obstacle_status.append(obstacle_status) #list of status of fire at each obstacle in a patch for all patches
                count+=1
        self.draw()

    def ignite_area(self, i, j):
        if (self.patch_obstacle_status[i][j] == False):
            for k in range(len(self.patch_obstacle_coordinates[i][j])):
                coordinate = self.patch_obstacle_coordinates[i][j][k]
                x = coordinate[0]
                y = coordinate[1]
                self.grid[x][y].color = RED
            self.patch_obstacle_status[i][j] = True
            self.burned_count += 1
            if (self.patch_obstacle_coordinates[i][j][0] == GREEN):
                self.intact_count -= 1
            self.patch_status[i] = True
            # self.draw()
        

    def ignite(self):
        i = random.randint(0, len(self.patch_centers) - 1)
        j = random.randint(0, len(self.patch_obstacle_coordinates[i]) - 1)
        while(self.patch_obstacle_status[i][j] == True):
            if (self.burned_count / self.total_count == 1):
                print("Forest completely burnt!")
                return False
            i = random.randint(0, len(self.patch_centers) - 1)
            j = random.randint(0, len(self.patch_obstacle_coordinates[i]) - 1)
        self.ignite_patch_center = i
        self.ignite_patch_obstacle = j
        fire_obs = i, j
        self.queue.append(fire_obs)
        if (self.patch_obstacle_status[i][j] == False):
            for k in range(len(self.patch_obstacle_coordinates[i][j])):
                coordinate = self.patch_obstacle_coordinates[i][j][k]
                x = coordinate[0]
                y = coordinate[1]
                self.grid[x][y].color = RED
            self.patch_obstacle_status[i][j] = True
            self.burned_count += 1
            self.intact_count -= 1
            self.patch_status[i] = True
            self.draw()
            threading.Timer(10, self.ignite).start()
            threading.Timer(10, self.simulate_fire_spread).start()

    def simulate_fire_spread(self):
        x = self.ignite_patch_center
        y = self.ignite_patch_obstacle
        if (self.patch_obstacle_status[x][y] == True):
            radius = self.fire_spread_dist
            patch_dist = math.sqrt(2) * self.fire_spread_dist
            patch_index = []
            for i in range(len(self.patch_centers)):
                if (algorithms.calc_dist(self.patch_centers[x], self.patch_centers[i]) < patch_dist):
                    patch_index.append(i)
            for i in patch_index:
                for j in range(len(self.patch_obstacle_coordinates[i])):
                    for k in range(len(self.patch_obstacle_coordinates[i][j])):
                        if (algorithms.calc_dist(self.patch_obstacle_coordinates[i][j][k], self.patch_obstacle_coordinates[x][y][0]) <= radius):
                            self.ignite_area(i, j)
                            # indices = i, j
                            fire_obs = i, j 
                            self.queue.append(fire_obs)
                            break
            # threading.Timer(3, self.ignite).start()
            self.draw()
            # threading.Timer(5, self.simulate_fire_spread).start()

    def simulate_burning_world(self):
        self.ignite()
        # threading.Timer(10, self.simulate_fire_spread).start()

    def extinguish(self, robot):
        for fire in range(len(self.queue)):
            # print(len(self.queue))
            # print(fire)
            element = self.queue[fire]
            i = element[0]
            j = element[1]
            for k in range(len(self.patch_obstacle_coordinates[i][j])):
                point = self.patch_obstacle_coordinates[i][j][k]
                if (algorithms.calc_dist(point, robot.position) <= 10):
                    for l in range(len(self.patch_obstacle_coordinates[i][j])):
                        m = self.patch_obstacle_coordinates[i][j][l][0]
                        n = self.patch_obstacle_coordinates[i][j][l][1]
                        self.grid[m][n].color = BLACK
                    self.patch_obstacle_status[i][j] = False
                    self.extinguished_count += 1
                    break
        # self.draw()

                        

                

    def fight_fire(self):
        parameters = []
        pygame.display.set_caption('A-Star Planner')
        robot = spawn_robot(self.grid)
        self.draw()
        t_start = time.time()
        t_end = t_start + 360
        while(time.time() < t_end):
            # print(len(self.queue))
            print(time.time() - t_start)
            if (len(self.queue) != 0):
                start = robot.position[0], robot.position[1], robot.heading
                fire_obs = self.queue[0]
                goal = self.patch_obstacle_coordinates[fire_obs[0]][fire_obs[1]][0]
                if (self.patch_obstacle_status[fire_obs[0]][fire_obs[1]] == True):
                    time_counter_start = time.time()
                    algorithms.AStar(self, robot, start, goal)
                    time_counter_end = time.time()
                    AStar_loop_time = time_counter_end - time_counter_start
                    self.AStar_time += AStar_loop_time
                    # algorithms.PRM(self, robot, start, goal)
                self.queue.popleft()
                self.reinitialize()
            while (len(self.queue) == 0):
                continue
        print("Simulation Complete")
        parameter = (self.intact_count / self.total_count), (self.extinguished_count / self.burned_count)
        parameters.append(parameter)
        threading.Timer(10, self.ignite).cancel()
        threading.Timer(7, self.simulate_fire_spread).cancel()
        print(parameter)
        print(self.AStar_time)

        time.sleep(2)

        self.reset_world()
        pygame.display.set_caption('PRM Planner')
        time.sleep(3)
        robot.clear_robot(self.grid)
        robot = spawn_robot(self.grid)
        self.draw()
        t_start = time.time()
        t_end = t_start + 360
        while(time.time() < t_end):
            # print(len(self.queue))
            if (len(self.queue) != 0):
                start = robot.position[0], robot.position[1], robot.heading
                fire_obs = self.queue[0]
                goal = self.patch_obstacle_coordinates[fire_obs[0]][fire_obs[1]][0]
                if (self.patch_obstacle_status[fire_obs[0]][fire_obs[1]] == True):
                    # algorithms.AStar(self, robot, start, goal)
                    time_counter_start = time.time()
                    algorithms.PRM(self, robot, start, goal)
                    time_counter_end = time.time()
                    PRM_loop_time = time_counter_end - time_counter_start
                    self.PRM_time += PRM_loop_time
                self.queue.popleft()
                self.reinitialize()
            while (len(self.queue) == 0):
                continue
        print("Simulation Complete")
        parameter = (self.intact_count / self.total_count), (self.extinguished_count / self.burned_count)
        parameters.append(parameter)
        print(parameters)
        print("AStar_Time, self.PRM_time")
        print(self.AStar_time, self.PRM_time)
        # success = algorithms.PRM(self, robot, start, goal)

def main():
    my_world = World(SIZE, ROWS, CONFIGURATION_SPACE, VERTICES, EDGES, OBSTACLE_ROWS, PATCH_DENSITY, FOREST_DENSITY)
    my_world.draw_world()
    my_world.simulate_burning_world()
    my_world.fight_fire()
    print("Done")
    run = True
    while run:
        # my_world.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
    pygame.quit()

main()      

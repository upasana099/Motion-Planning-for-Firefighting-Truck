from wumpus import *
from algorithms import *
from global_var import *
from world import *



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

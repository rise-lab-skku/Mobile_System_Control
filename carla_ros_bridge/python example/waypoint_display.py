#!/usr/bin/env python3

import carla

def main():

    client = carla.Client("localhost", 2000)
    client.set_timeout(10)

    world = client.get_world()

    spawn_points = world.get_map().get_spawn_points()
    print('Total waypoint: ', len(spawn_points), '\n')

    num = 0

    for point in spawn_points:
        print(point)
        print(type(point))
        order = chr(int(num / 10) + 48) + chr(num - int(num/10) * 10 + 48)
        world.debug.draw_string(point.location, order, draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=600.0, persistent_lines=True)
        print(order, ': ', point.location)
        num += 1
    
    print('\nBye! ')

if __name__ == '__main__':

    main()


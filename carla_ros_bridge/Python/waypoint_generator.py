#!/usr/bin/env python3

import carla
import time

from agents.navigation.global_route_planner import GlobalRoutePlanner

def main():

    client = carla.Client("localhost", 2000)
    client.set_timeout(10)

    world = client.get_world()

    amap = world.get_map()
    sampling_resolution = 3
    grp = GlobalRoutePlanner(amap, sampling_resolution)
    spawn_points = amap.get_spawn_points()
    print(len(spawn_points))
    a = 0
    b = 1
    lengh = 0
    for i in range(len(spawn_points)):
        for j in range(len(spawn_points) - i):
            aa = carla.Location(spawn_points[i].location)
            bb = carla.Location(spawn_points[j].location)
            w1 = grp.trace_route(aa, bb) 
            if len(w1) > lengh:
                lengh = len(w1)
                a = aa
                b = bb

    w1 = grp.trace_route(a, b) 
    
    w_temp = 0

    for w in w1:
        if w[0].transform.location == w_temp:
            continue
        else:
            world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
            color=carla.Color(r=0, g=0, b=255), life_time=20.0,
            persistent_lines=True)
            print(w[0].transform.location)
            w_temp = w[0].transform.location
            time.sleep(0.01)

if __name__ == '__main__':

    main()


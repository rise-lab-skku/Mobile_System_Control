#!/usr/bin/env python3

import carla
import time
from math import sqrt, pow, cos, pi

from agents.navigation.global_route_planner import GlobalRoutePlanner

def main():

    client = carla.Client("localhost", 2000)
    client.set_timeout(10)

    world = client.get_world()

    amap = world.get_map()
    grp = GlobalRoutePlanner(amap, 1)
    spawn_points = amap.get_spawn_points()
    print("Number of Spawn Points: ", len(spawn_points))

###################################################
################################################### 


    # Set Waypoint (in or out)
    track = "in"

    # Set Waypoint 
    a = 2
    b = 4
    c = a

    # Set Lon, Lat at Origin point (0,0,0)
    lat = 37.296664
    lon = 126.973627

    # Waypoint Distance [m]
    sampling_resolution = 0.5


###################################################
###################################################

    fxy = open("waypoint_" + track + "xy.csv","w+")
    fll = open("waypoint_" + track + ".csv","w+")

    grp = GlobalRoutePlanner(amap, sampling_resolution / 10)
    w1 = grp.trace_route(carla.Location(spawn_points[a].location), carla.Location(spawn_points[b].location)) 
    w2 = grp.trace_route(carla.Location(spawn_points[b].location), carla.Location(spawn_points[c].location)) 
    w_temp = w1[0][0].transform.location
    for w in w1 + w2:
        if w[0].transform.location == w_temp:
            continue
        elif sqrt(pow(w_temp.x-w[0].transform.location.x,2)+pow(w_temp.y-w[0].transform.location.y,2)) < sampling_resolution * 0.98:
            continue
        else:
            world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
            color=carla.Color(r=0, g=0, b=255), life_time=20.0,
            persistent_lines=True)
            print(w[0].transform.location.x,'\t', -w[0].transform.location.y,'\t', w[0].transform.location.z)
            fxy.write("%f\t%f\t%f\n" % (w[0].transform.location.x, -w[0].transform.location.y, w[0].transform.location.z))

            latitude    = -w[0].transform.location.y / (pi * 6378.135 / 180 * 1000) + lat
            longitude   = w[0].transform.location.x / (cos(lat * pi / 180) * pi * 6378.135 / 180 * 1000) + lon
            fll.write("%f\t%f\n" % (latitude, longitude))
            
            w_temp = w[0].transform.location
            time.sleep(0.01)
    fxy.close
    fll.close

if __name__ == '__main__':

    main()


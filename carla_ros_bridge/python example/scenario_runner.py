#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import math
import random
import time
import random

import json

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.sync = args.sync
        self.scenario = args.scenario
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.players = []
        self._actor_generation = args.generation
        self.cleannpc()
        self.spawn()
        self.constant_velocity_enabled = False

    def cleannpc(self):
        actor_list = self.world.get_actors()
        for actor in actor_list.filter('*npc*'):
            if actor.id != 1:
                actor.destroy()
            else:
                continue

    def spawn(self):
        
        spawn_points = self.map.get_spawn_points()

        with open('scenario/scenario_' + self.scenario + '.json', 'r') as f:
            npc_list = json.load(f)

        waypoint = spawn_points[1]

        # randid = random.randrange(10,50)

        print(npc_list["npc list"])

        for i, npc in enumerate(npc_list["npc list"]):
            blueprint = (get_actor_blueprints(self.world, 'erp42npc' + str(npc_list[npc]["speed"]), self._actor_generation)[0])
            blueprint.set_attribute('role_name', 'npc' + str(npc_list[npc]["speed"]))
            
            if npc_list[npc]["point"] < 0:
                waypoint.location.x = npc_list[npc]["x"]
                waypoint.location.y = npc_list[npc]["y"]
                waypoint.location.z = npc_list[npc]["z"]
                waypoint.rotation.roll = npc_list[npc]["roll"]
                waypoint.rotation.pitch = npc_list[npc]["pitch"]
                waypoint.rotation.yaw = npc_list[npc]["yaw"]
            else:
                waypoint = spawn_points[npc_list[npc]["point"]]
            
            temp = self.world.try_spawn_actor(blueprint, waypoint)
            # temp.set_attribute("id", str(10 + i))
            self.show_vehicle_telemetry = False
            if temp is not None:
                self.players.append(temp)

        for npc in self.players:
            npc.set_autopilot(False)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def StartAutopilot(self):
        for npc in self.players:
            npc.set_autopilot(True)

def spawn_loop(args):
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.destroy:
            actor_list = sim_world.get_actors()
            for actor in actor_list:
                if actor.id != 1:
                    actor.destroy()
                else:
                    continue
        else:
            world = World(sim_world, args)

            if args.sync:
                sim_world.tick()
            else:
                sim_world.wait_for_tick()

            while True:
                user_input = input('Press key to play scenario:\n\tenter\tstart\n\tr\trespawn')
                if user_input == '':
                    world.StartAutopilot()
                elif user_input == 'r':
                    world.cleannpc()
                    world.spawn()
                else:
                    continue
                if args.sync:
                    sim_world.tick()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if world is not None:
            world.cleannpc()

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='4',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '-s', '--scenario',
        default='1',
        type=str,
        help='scenario number')
    argparser.add_argument(
        '-d', '--destroy',
        default=False,
        type=bool,
        help='destroy all actors')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)
    try:
        spawn_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':

    main()

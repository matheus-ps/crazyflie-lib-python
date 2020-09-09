# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyfles can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.points import Point

currPos = {}


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            #print(scf.cf.link_uri[-1] + " {} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    #scf.cf.param.set_value('p2p.p2pEnable', '1')
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')
    scf.cf.param.set_value('avoidance.enTcas2', '1')
    scf.cf.param.set_value('commander.eneableAP', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

def positionCallback(timestamp, data, logconf, uri):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    #vx = data['stateEstimate.vx']
    #vy = data['stateEstimate.vy']
    #vz = data['stateEstimate.vz']
    #print("positionCallback")
    print(uri[-1] + " %.2f %.2f %.2f" %  (x, y, z))
    currPos[uri] = Point(x, y, z)

def startPositionPrinting(scf):
    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9':
    #    return
    logConf = LogConfig(name='Position', period_in_ms=200)
    logConf.add_variable('stateEstimate.x', 'float')
    logConf.add_variable('stateEstimate.y', 'float')
    logConf.add_variable('stateEstimate.z', 'float')
    logConf.add_variable('stateEstimate.vx', 'float')
    logConf.add_variable('stateEstimate.vy', 'float')
    logConf.add_variable('stateEstimate.vz', 'float')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: positionCallback(timestamp, data, logconf, scf.cf.link_uri))
    logConf.start()

def lastCallback(timestamp, data, logconf, uri):
    x = data['last.x']
    y = data['last.y']
    z = data['last.z']
    vx = data['last.vx']
    vy = data['last.vy']
    vz = data['last.vz']
    coll = data['last.cf']
    # Print
    print("%.0f || %.2f %.2f %.2f || %.2f %.2f %.2f" %  (coll, x, y, z, vx, vy, vz))

def startLastPrinting(scf):
    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9':
    #    return
    logConf = LogConfig(name='Last', period_in_ms=200)
    logConf.add_variable('last.x', 'float')
    logConf.add_variable('last.y', 'float')
    logConf.add_variable('last.z', 'float')
    logConf.add_variable('last.vx', 'float')
    logConf.add_variable('last.vy', 'float')
    logConf.add_variable('last.vz', 'float')
    logConf.add_variable('last.cf', 'uint8_t')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: lastCallback(timestamp, data, logconf, scf.cf.link_uri))
    logConf.start()

def P2PCallback(timestamp, data, logconf, uri):
    px = data['p2pmsg.px']
    py = data['p2pmsg.py']
    pz = data['p2pmsg.pz']
    sid = data['p2pmsg.sid']
    #print('P2PCallback')
    print(uri[-1] + " %.2f %.2f %.2f %.0f" % (px, py, pz, sid))

def startP2PPrinting(scf):
    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9':
    #    return
    logConf = LogConfig(name='P2P', period_in_ms=200)
    #logConf.add_variable('p2pmsg.vx', 'float')
    #logConf.add_variable('p2pmsg.vy', 'float')
    #logConf.add_variable('p2pmsg.vz', 'float')
    logConf.add_variable('p2pmsg.px', 'float')
    logConf.add_variable('p2pmsg.py', 'float')
    logConf.add_variable('p2pmsg.pz', 'float')
    #logConf.add_variable('p2pmsg.rot', 'float')
    #logConf.add_variable('p2pmsg.srssi', 'uint8_t')
    logConf.add_variable('p2pmsg.sid', 'uint8_t')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: P2PCallback(timestamp, data, logconf, scf.cf.link_uri))
    logConf.start()

def debugCallback(timestamp, data, logconf, uri):
    px = data['debu.stop']
    #print('P2PCallback')
    print(" %.0f" % (px))

def startDebugPrinting(scf):
    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9':
    #    return
    logConf = LogConfig(name='Debug', period_in_ms=100)
    logConf.add_variable('debu.stop', 'uint8_t')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: debugCallback(timestamp, data, logconf, scf.cf.link_uri))
    logConf.start()

def run_shared_sequence(scf):
    activate_mellinger_controller(scf, False)

    box_size = 0.7
    flight_time = 2

    commander = scf.cf.high_level_commander
    time.sleep(flight_time)

    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7EB' or scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7EA' or scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9' or scf.cf.link_uri == 'usb://0':
    #    for i in range(200):
    #        time.sleep(1)
    
    print("Takeoff")
    commander.takeoff(1.0, flight_time)
    time.sleep(0.5)

    #print("Go")
    #commander.go_to(0.60, 0.65, 1.0, 0, flight_time*2, relative=False)
    #time.sleep(20)

    print("Land")
    commander.land(0.0, flight_time)
    time.sleep(0.5)

    commander.stop()


uris = {
    #'radio://0/80/2M/E7E7E7E7EA',
    'radio://0/80/2M/E7E7E7E7E9',
    #'usb://0',
    # Add more URIs if you want more copters in the swarm
}

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
        #swarm.parallel_safe(startPositionPrinting)
        #swarm.parallel_safe(startP2PPrinting)
        #swarm.parallel_safe(startLastPrinting)
        #swarm.parallel_safe(startDebugPrinting)
        swarm.parallel_safe(run_shared_sequence)
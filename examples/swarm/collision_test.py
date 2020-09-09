# -*- coding: utf-8 -*-
#
# TCAS II MODULE - MASTER THESIS BY
# Matheus Pedroso Sanches - MIEEC 2020
# Faculdade de Engenharia da Universidade do Porto - Portugal
# Altran Portugal SA
#
# 🅼🅰🆃🅷🅴🆄🆂
# 🄰🄻🅃🅁🄰🄽 & 🄵🄴🅄🄿

import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

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
    wait_for_position_estimator(scf)

def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')
    scf.cf.param.set_value('avoidance.enTcas2', '1')
    scf.cf.param.set_value('p2p.p2pEnable', '1')
    #scf.cf.param.set_value('commander.enableAP', '1')

def positionCallback(timestamp, data, logconf, uri):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    print(uri[-1] + " %.2f %.2f %.2f" %  (x, y, z))

def startPositionPrinting(scf):
    #if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E9':
    #    return
    logConf = LogConfig(name='Position', period_in_ms=200)
    logConf.add_variable('stateEstimate.x', 'float')
    logConf.add_variable('stateEstimate.y', 'float')
    logConf.add_variable('stateEstimate.z', 'float')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: positionCallback(timestamp, data, logconf, scf.cf.link_uri))
    logConf.start()

def P2PCallback(timestamp, data, logconf, uri):
    px = data['p2pmsg.px']
    py = data['p2pmsg.py']
    pz = data['p2pmsg.pz']
    sid = data['p2pmsg.sid']
    print(uri[-1] + " %.2f %.2f %.2f %.0f" % (px, py, pz, sid))

def startP2PPrinting(scf):
    if scf.cf.link_uri == uris["I"]:
        return
    logConf = LogConfig(name='P2P', period_in_ms=200)
    #logConf.add_variable('p2pmsg.vx', 'float')
    #logConf.add_variable('p2pmsg.vy', 'float')
    #logConf.add_variable('p2pmsg.vz', 'float')
    logConf.add_variable('p2pmsg.px', 'float')
    logConf.add_variable('p2pmsg.py', 'float')
    logConf.add_variable('p2pmsg.pz', 'float')
    #logConf.add_variable('p2pmsg.srssi', 'uint8_t')
    logConf.add_variable('p2pmsg.sid', 'uint8_t')

    scf.cf.log.add_config(logConf)
    logConf.data_received_cb.add_callback(lambda timestamp, data, logconf: P2PCallback(timestamp, data, logconf, scf.cf.link_uri))
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
    print(uri[-1] + " %.0f || %.2f %.2f %.2f || %.2f %.2f %.2f" %  (coll, x, y, z, vx, vy, vz))

def startLastPrinting(scf):
    #if scf.cf.link_uri == uris["VI"]:
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

def run_shared_sequence(scf):

    flight_time = 8

    commander = scf.cf.high_level_commander
    time.sleep(5)

    #if scf.cf.link_uri == uris['I']: #or scf.cf.link_uri == uris['I']:
    #    for i in range(200):
    #        time.sleep(1)
    
    print("Takeoff")
    commander.takeoff(0.84, 2)
    time.sleep(4)

    scf.cf.param.set_value('commander.enableAP', '1')

    print("Head-on collision")
    if scf.cf.link_uri == uris['IV']:
        commander.go_to(0.50, 0.55, 0.84, 0, flight_time, relative=False)
    if scf.cf.link_uri == uris['I']:
        commander.go_to(1.82, 1.85, 0.84, 0, flight_time, relative=False)
    time.sleep(15)

    print("Close motion")
    if scf.cf.link_uri == uris['IV']:
        commander.go_to(0.50, 1.85, 0.84, 0, flight_time, relative=False)
    if scf.cf.link_uri == uris['I']:
        commander.go_to(0.50, 0.55, 0.84, 0, flight_time, relative=False)
    time.sleep(10)

    print("Converging collision")
    if scf.cf.link_uri == uris['IV']:
        commander.go_to(1.82, 0.55, 0.84, 0, flight_time, relative=False)
    if scf.cf.link_uri == uris['I']:
        commander.go_to(1.82, 1.85, 0.84, 0, flight_time, relative=False)
    time.sleep(15)

    print("Close motion")
    if scf.cf.link_uri == uris['IV']:
        commander.go_to(1.82, 1.85, 0.84, 0, flight_time, relative=False)
    if scf.cf.link_uri == uris['I']:
        commander.go_to(0.50, 0.55, 0.84, 0, flight_time, relative=False)
    time.sleep(10)

    scf.cf.param.set_value('commander.enableAP', '0')
    scf.cf.param.set_value('avoidance.enTcas2', '0')
    scf.cf.param.set_value('p2p.p2pEnable', '0')

    print("Land")
    commander.land(0.0, 2)
    time.sleep(4)

    commander.stop()


uris = {
    'I'     :   'radio://0/80/2M/E7E7E7E7E3', # I
    #'II'    :   'radio://0/80/2M/E7E7E7E7E5', # II
    #'III'   :   'radio://0/80/2M/E7E7E7E7E8', # III
    'IV'    :   'radio://0/80/2M/E7E7E7E7E9', # IV
    #'V'     :   'radio://0/80/2M/E7E7E7E7EA', # V
    #'VI'    :   'radio://0/80/2M/E7E7E7E7EB', # VI
    #'USB'   :   'usb://0',                    # USB
}

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(set(uris.values()), factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(startLastPrinting)
        swarm.parallel_safe(run_shared_sequence)
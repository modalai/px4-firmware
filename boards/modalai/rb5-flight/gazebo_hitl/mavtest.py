#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (PositionNedYaw, VelocityNedYaw, OffboardError)
import os
import time

async def run():

    drone_timeout = 5.0

    drone = System()
    await asyncio.wait_for(drone.connect(system_address="udp://:14550"), timeout = drone_timeout)

    print("Waiting for drone to connect...")

    connection_timer = time.time()
    connection_diff = 0
    async for state in drone.core.connection_state():
        connection_diff = time.time() - connection_timer
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
        if(connection_diff > 10):
            print("Device is taking too long to connect, check simulation")
            return 

    print("Waiting for drone to have a global position estimate...")

    gps_timer = time.time()
    gps_diff = 0
    async for health in drone.telemetry.health():
        gps_diff = time.time() - gps_timer
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
        if(gps_diff > 10):
            print("Device unable to grab position lock, check simulation")
            return

    arm_result = try_except_call(await asyncio.wait_for(drone.action.arm(), timeout = drone_timeout), "-- Arming")
    
    offboard_position_set_result = try_except_call(await asyncio.wait_for(drone.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0),VelocityNedYaw(0.0,0.0,0.0,0.0)), timeout = drone_timeout), "-- Setting initial setpoint")
    offboard_start_result = try_except_call(await asyncio.wait_for(drone.offboard.start(), timeout = drone_timeout), "-- Starting offboard")
    offboard_position_set_1_result = try_except_call(await asyncio.wait_for(drone.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0),VelocityNedYaw(0.0,0.0,1.0,0.0)), timeout = drone_timeout), "-- Go 5m UP within local coordinate system")
    await asyncio.sleep(10)
    
    offboard_position_set_2_result = try_except_call(await asyncio.wait_for(drone.offboard.set_position_velocity_ned(PositionNedYaw(5.0, 0.0, 0.0, 0.0),VelocityNedYaw(1.0,0.0,0.0,0.0)), timeout = drone_timeout), "-- Go 5m North within local coordinate system")
    await asyncio.sleep(10)

    offboard_stop_result = try_except_call(await asyncio.wait_for(drone.offboard.stop(), timeout = drone_timeout), "-- Stopping offboard")
    land_result = try_except_call(await asyncio.wait_for(drone.action.land(), timeout = drone_timeout), "-- landing")

def try_except_call(action, success_print):
    try:
        action
        print(success_print)
        return True
    except Exception as exception:
        print("Exception: {}".format(type(exception).__name__))
        print("Exception message: {}".format(exception))
        return False

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

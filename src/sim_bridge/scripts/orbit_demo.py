#!/usr/bin/env python3
"""
Simple MAVSDK Offboard orbit demo for PX4 SITL.

This script:
  - connects to PX4 (SITL) over UDP
  - arms the drone
  - enters offboard mode
  - flies a flat circle around (0,0) at fixed altitude
  - then lands

It is NOT yet wired to ROS 2; it's a standalone sanity check that
PX4 SITL + MAVSDK Offboard control are working.
"""

import asyncio
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


RADIUS_M = 5.0
ALTITUDE_M = -3.0  # NED frame: negative Z is "up"
ORBIT_TIME_S = 30.0


async def run():
    drone = System()
    # Default PX4 SITL MAVSDK address; adjust if needed
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4!")
            break

    print("Waiting for position estimate...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position OK")
            break

    print("Arming...")
    await drone.action.arm()

    # Prime offboard with an initial setpoint
    print("Setting initial setpoint and starting offboard...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, ALTITUDE_M, 0.0)
    )

    try:
        await drone.offboard.start()
        print("Offboard mode started.")
    except OffboardError as e:
        print(f"Offboard start failed: {e._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Circle around origin (0,0) at fixed altitude, yaw facing center
    steps = 360
    total_time = ORBIT_TIME_S
    dt = total_time / steps

    for i in range(steps):
        theta = 2 * math.pi * (i / steps)
        north = RADIUS_M * math.cos(theta)
        east = RADIUS_M * math.sin(theta)
        # yaw that keeps nose pointing toward the center (0,0)
        yaw_deg = math.degrees(math.atan2(-east, -north))

        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, ALTITUDE_M, yaw_deg)
        )
        await asyncio.sleep(dt)

    print("Stopping offboard and landing...")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        print(f"Offboard stop failed: {e._result.result}")

    await drone.action.land()
    print("Landing command sent. Done.")


def main():
    asyncio.run(run())


if __name__ == "__main__":
    main()

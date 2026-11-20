"""Drive the cars, fly the drones and walk the Go2s in the Blocks Hybrid example."""

import sys
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from math import hypot
from threading import Event

import cosysairsim as airsim


CARS = ("Car1", "Car2", "Car3", "Husky1")
DRONES = {"Drone1": 1, "Drone2": -1}
PORT = 41451


def drive_car(vehicle_name, stop):
    car = airsim.CarClient(port=PORT, timeout_value=15)
    car.getCarState(vehicle_name=vehicle_name)
    start = car.simGetVehiclePose(vehicle_name=vehicle_name).position
    car.enableApiControl(True, vehicle_name=vehicle_name)
    try:
        print(f"{vehicle_name} driving continuously. Press Ctrl+C to stop.", flush=True)
        car.setCarControls(airsim.CarControls(throttle=0.50), vehicle_name=vehicle_name)
        while not stop.wait(3):
            speed = car.getCarState(vehicle_name=vehicle_name).speed
            position = car.simGetVehiclePose(vehicle_name=vehicle_name).position
            displacement = hypot(position.x_val - start.x_val, position.y_val - start.y_val)
            print(f"{vehicle_name} speed: {speed:.2f} m/s, displacement: {displacement:.2f} m", flush=True)
    finally:
        car.setCarControls(airsim.CarControls(brake=1, handbrake=vehicle_name == "Husky1"),
                           vehicle_name=vehicle_name)
        time.sleep(1)
        car.enableApiControl(False, vehicle_name=vehicle_name)


def fly_drone(vehicle_name, sideways_speed, stop):
    drone = airsim.MultirotorClient(port=PORT, timeout_value=35)
    drone_enabled = False
    drone_armed = False
    cleanup_errors = []

    try:
        initial_state = drone.getMultirotorState(vehicle_name=vehicle_name)
        initial_z = initial_state.kinematics_estimated.position.z_val
        drone.enableApiControl(True, vehicle_name=vehicle_name)
        drone_enabled = True
        if not drone.armDisarm(True, vehicle_name=vehicle_name):
            raise RuntimeError(f"{vehicle_name} could not arm")
        drone_armed = True
        print(f"{vehicle_name} taking off", flush=True)
        # Climb relative to the settled position on the Blocks floor.
        if not drone.moveByVelocityAsync(0, 0, -1, 2.5, vehicle_name=vehicle_name).get():
            raise RuntimeError(f"{vehicle_name} climb was interrupted")
        state = drone.getMultirotorState(vehicle_name=vehicle_name)
        climb = initial_z - state.kinematics_estimated.position.z_val
        if climb < 1 or state.landed_state != airsim.LandedState.Flying:
            raise RuntimeError(f"{vehicle_name} did not lift clear of the ground (climb {climb:.2f} m)")

        if not stop.is_set():
            print(f"{vehicle_name} moving sideways for 3 seconds", flush=True)
            if not drone.moveByVelocityAsync(0, sideways_speed, 0, 3, vehicle_name=vehicle_name).get():
                raise RuntimeError(f"{vehicle_name} did not complete the move")
    finally:
        if drone_enabled:
            try:
                if drone_armed:
                    if not drone.landAsync(timeout_sec=30, vehicle_name=vehicle_name).get():
                        raise RuntimeError("landing did not complete")
                    if not drone.armDisarm(False, vehicle_name=vehicle_name):
                        raise RuntimeError("disarming failed")
                drone.enableApiControl(False, vehicle_name=vehicle_name)
            except Exception as exc:
                cleanup_errors.append(f"{vehicle_name} cleanup: {exc}")
        for message in cleanup_errors:
            print(message, file=sys.stderr)

    if cleanup_errors:
        raise RuntimeError("Vehicle cleanup was incomplete")
    print(f"{vehicle_name} landed")


def run_demo():
    client = airsim.VehicleClient(port=PORT, timeout_value=15)
    try:
        active = set(client.listVehicles())
        dogs = client.listGo2Vehicles()
    finally:
        client.client.close()
    active.difference_update(dogs)
    cars = [name for name in CARS if name in active]
    drones = {name: speed for name, speed in DRONES.items() if name in active}
    if dogs:
        import go2_bridge
        dog_args = go2_bridge.parse_args(["--port", str(PORT)])

    stop = Event()
    count = len(cars) + len(drones) + len(dogs)
    if not count:
        raise RuntimeError("No demo vehicles are active")
    with ThreadPoolExecutor(max_workers=count) as pool:
        jobs = {pool.submit(drive_car, name, stop): name for name in cars}
        for name, speed in drones.items():
            jobs[pool.submit(fly_drone, name, speed, stop)] = name
        for name in dogs:
            jobs[pool.submit(go2_bridge.run_go2, dog_args, name, stop)] = name
        try:
            while jobs:
                completed, _ = wait(jobs, timeout=0.25, return_when=FIRST_COMPLETED)
                for job in completed:
                    name = jobs.pop(job)
                    try:
                        job.result()
                    except Exception as exc:
                        print(f"{name}: {exc}", file=sys.stderr, flush=True)
        finally:
            stop.set()


if __name__ == "__main__":
    try:
        run_demo()
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        sys.exit(f"Hybrid demo failed: {exc}")

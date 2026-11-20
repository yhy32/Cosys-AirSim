"""Drive Go2 pawns in AirSim Go2 or Hybrid mode from the MuJoCo model."""

import argparse
import math
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event

import msgpackrpc
import mujoco
import numpy as np

DEFAULT_SCENE = Path("C:/BUPT/mujoco_menagerie/unitree_go2/scene.xml")

LEG_PHASE = np.array([0.0, 0.0, 0.0, math.pi, math.pi, math.pi, math.pi, math.pi, math.pi, 0.0, 0.0, 0.0])
KP = np.array([40, 50, 50] * 4, dtype=float)
KD = np.array([1.0, 1.2, 1.2] * 4, dtype=float)
HOME = np.array([0.0, 0.9, -1.8] * 4)

A_THIGH = 0.35
A_CALF = 0.60
FREQ = 1.6
SETTLE = 0.5
RAMP = 2.0
START_PHASE = math.pi / 3
TURN_DURATION = 3.0
TURN_BLEND = 0.5
LEFT_THIGH_INDICES = (1, 7)
UE_OBSTACLE_CONTACT = 2


def add_robot_collision_shell(spec):
    """Approximate the visible Go2 shell without changing native foot contacts."""
    shell = dict(contype=0, conaffinity=UE_OBSTACLE_CONTACT, mass=0, group=3)
    base = spec.body("base")
    base.add_geom(name="go2_shell_body", type=mujoco.mjtGeom.mjGEOM_BOX,
                  pos=(0.055, 0, -0.004), size=(0.183, 0.098, 0.093), **shell)
    base.add_geom(name="go2_shell_head", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=(0.222, 0, 0.005, 0.281, 0, 0.005), size=(0.057, 0, 0), **shell)
    for leg in ("FL", "FR", "RL", "RR"):
        hip_center = -0.011 if leg.startswith("F") else 0.011
        thigh_y = -0.02 if leg.endswith("L") else 0.02
        spec.body(f"{leg}_hip").add_geom(
            name=f"go2_shell_{leg}_hip", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=(hip_center - 0.009, 0, 0, hip_center + 0.009, 0, 0),
            size=(0.050, 0, 0), **shell,
        )
        spec.body(f"{leg}_thigh").add_geom(
            name=f"go2_shell_{leg}_thigh", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=(0, thigh_y, 0, 0, thigh_y, -0.177),
            size=(0.050, 0, 0), **shell,
        )
        spec.body(f"{leg}_calf").add_geom(
            name=f"go2_shell_{leg}_calf", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=(0.003, 0, 0, 0.003, 0, -0.205),
            size=(0.031, 0, 0), **shell,
        )


class NearbyObstaclePool:
    """Mirror nearby UE obstacles as kinematic boxes in MuJoCo."""

    PARKED = (0, 0, -10000)
    INTEGRATION_STATE = mujoco.mjtState.mjSTATE_INTEGRATION

    def __init__(self, spec, capacity, enter_radius, clearance=0.0):
        self.spec = spec
        self.enter_radius = enter_radius
        self.clearance = clearance
        self.boxes = []
        self.sizes = [(0.05, 0.05, 0.05)] * capacity
        self.slots = {}
        for index, size in enumerate(self.sizes):
            body = spec.worldbody.add_body(
                name=f"go2_ue_obstacle_{index}", mocap=True, pos=self.PARKED
            )
            self.boxes.append(body.add_geom(
                name=f"go2_ue_box_{index}", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=size, contype=1 | UE_OBSTACLE_CONTACT, conaffinity=0,
            ))

    def update(self, model, data, rows, home_position):
        active = {
            int(row[0]): row for row in rows
            if row[-1] <= self.enter_radius or int(row[0]) in self.slots
        }
        # Keep the nearest boxes when the budget is exhausted.
        active = dict(sorted(active.items(), key=lambda pair: pair[1][-1])[:len(self.boxes)])
        self.slots = {key: slot for key, slot in self.slots.items() if key in active}
        free = set(range(len(self.boxes))) - set(self.slots.values())
        needs_compile = False
        for key, row in active.items():
            # Optional horizontal padding preserves the step's walking height.
            size = (
                max(0.001, round(row[4] + self.clearance, 3)),
                max(0.001, round(row[5] + self.clearance, 3)),
                max(0.001, round(row[6], 3)),
            )
            if key not in self.slots:
                matching = [slot for slot in free if self.sizes[slot] == size]
                slot = min(matching or free)
                self.slots[key] = slot
                free.remove(slot)
            slot = self.slots[key]
            if size != self.sizes[slot]:
                self.sizes[slot] = size
                self.boxes[slot].size = size
                needs_compile = True

        if needs_compile:
            # Resizing a compiled geom would leave collision bounds stale.
            # Keep the pool topology and restore the running simulation state.
            state = np.empty(mujoco.mj_stateSize(model, self.INTEGRATION_STATE))
            mujoco.mj_getState(model, data, state, self.INTEGRATION_STATE)
            model = self.spec.compile()
            data = mujoco.MjData(model)
            mujoco.mj_setState(model, data, state, self.INTEGRATION_STATE)

        floor_offset = (home_position[0], home_position[1], 0)
        for index in range(len(self.boxes)):
            mocap = model.body(f"go2_ue_obstacle_{index}").mocapid[0]
            data.mocap_pos[mocap] = self.PARKED
        for key, row in active.items():
            mocap = model.body(f"go2_ue_obstacle_{self.slots[key]}").mocapid[0]
            data.mocap_pos[mocap] = np.asarray(row[1:4]) + floor_offset
            data.mocap_quat[mocap] = row[7:11]
        mujoco.mj_forward(model, data)
        return model, data

    def contact_ids(self, model, data):
        obstacles = {
            model.geom(f"go2_ue_box_{slot}").id: key for key, slot in self.slots.items()
        }
        return {
            obstacles[geom] for contact in data.contact
            for geom in contact.geom if geom in obstacles
        }


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--port", type=int, default=41451)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--vehicle", help="Drive only this named Go2")
    target.add_argument("--all", action="store_true", help="Drive every Go2 (default)")
    parser.add_argument("--maneuver", choices=("walk", "turn-walk"),
                        help="Override the vehicle's demonstration maneuver")
    parser.add_argument("--send-hz", type=float, default=100.0)
    parser.add_argument("--obstacle-radius", type=float, default=0.65, help="Tagged MuJoCo obstacle radius in meters (0 disables tagged boxes)")
    parser.add_argument("--obstacle-hz", type=float, default=10.0, help="UE obstacle query rate")
    parser.add_argument("--obstacle-limit", type=int, default=32, help="Maximum active collision boxes")
    parser.add_argument("--obstacle-clearance", type=float, default=0.0, help="Optional horizontal padding in meters")
    parser.add_argument("--debug-obstacles", action="store_true", help="Report UE body hits, active tagged boxes and MuJoCo contacts")
    args = parser.parse_args(argv)

    if not args.scene.is_file():
        parser.error(f"MuJoCo scene not found: {args.scene}")
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if not math.isfinite(args.send_hz) or args.send_hz <= 0:
        parser.error("--send-hz must be positive and finite")
    if not 0 <= args.obstacle_radius <= 20:
        parser.error("--obstacle-radius must be between 0 and 20 meters")
    if not math.isfinite(args.obstacle_hz) or args.obstacle_hz <= 0:
        parser.error("--obstacle-hz must be positive and finite")
    if not 1 <= args.obstacle_limit <= 128:
        parser.error("--obstacle-limit must be between 1 and 128")
    if not 0 <= args.obstacle_clearance <= 0.25:
        parser.error("--obstacle-clearance must be between 0 and 0.25 meters")
    return args


def main():
    args = parse_args()
    stop = Event()
    if args.vehicle is not None:
        run_go2(args, args.vehicle, stop)
        return

    client = rpc_client(args.port)
    try:
        vehicles = client.call("listGo2Vehicles")
    finally:
        client.close()
    if not vehicles:
        raise RuntimeError("No Go2 vehicles are available")

    with ThreadPoolExecutor(max_workers=len(vehicles)) as pool:
        jobs = [pool.submit(run_go2, args, vehicle, stop) for vehicle in vehicles]
        try:
            for job in jobs:
                job.result()
        finally:
            stop.set()


def rpc_client(port):
    return msgpackrpc.Client(
        msgpackrpc.Address("127.0.0.1", port), timeout=10,
        pack_encoding="utf-8", unpack_encoding="utf-8",
    )


def run_go2(args, vehicle, stop):
    client = rpc_client(args.port)
    try:
        stream_go2(args, vehicle, stop, client)
    finally:
        client.close()


def apply_body_feedback(model, data, home_position, feedback):
    offset, rotation, *normals = feedback
    data.qpos[:3] = home_position + offset
    if rotation:
        data.qpos[3:7] = rotation
        data.qvel[3:6] = 0

    # Both the free joint's linear velocity and the returned normals use the spawn frame.
    velocity = data.qvel[:3]
    for index, normal in enumerate(np.asarray(normals)):
        velocity -= min(0.0, np.dot(velocity, normal)) * normal
        # At an acute corner, projecting onto one wall can point back into the other.
        for previous in normals[:index]:
            if np.dot(velocity, previous) < -1e-7:
                edge = np.cross(previous, normal)
                velocity[:] = edge * (np.dot(velocity, edge) / max(np.dot(edge, edge), 1e-12))
    if any(np.dot(velocity, normal) < -1e-7 for normal in normals):
        velocity[:] = 0
    mujoco.mj_forward(model, data)


def stream_go2(args, vehicle, stop, client):
    maneuver = args.maneuver or ("turn-walk" if vehicle == "go2_2" else "walk")
    obstacles = None
    if args.obstacle_radius:
        spec = mujoco.MjSpec.from_file(str(args.scene))
        add_robot_collision_shell(spec)
        obstacles = NearbyObstaclePool(spec, args.obstacle_limit, args.obstacle_radius, args.obstacle_clearance)
        model = spec.compile()
    else:
        model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    data.qpos[:] = model.key_qpos[0]
    # UE move requests, collision feedback and nearby obstacles all use this same spawn frame.
    home_position = data.qpos[:3].copy()
    angles, offset, orientation = client.call("getGo2State", vehicle)
    data.qpos[:3] = home_position + offset
    data.qpos[3:7] = orientation
    data.qpos[7:19] = angles
    mujoco.mj_forward(model, data)
    starting_angles = data.qpos[7:19].copy()
    dt = model.opt.timestep
    steps = int(args.duration / dt)
    send_every = max(1, int(round(1.0 / (args.send_hz * dt))))
    query_every = max(1, int(round(1.0 / (args.obstacle_hz * dt))))
    ctrl_lo = model.actuator_ctrlrange[:, 0].copy()
    ctrl_hi = model.actuator_ctrlrange[:, 1].copy()

    wall_start = time.perf_counter()
    sent = 0
    active_ids = set()
    contact_ids = set()
    last_body_hit = -math.inf
    print(f"{vehicle}: {maneuver}", flush=True)
    for i in range(steps):
        if stop.is_set():
            return
        if obstacles is not None and i % query_every == 0:
            query_radius = args.obstacle_radius * 1.25
            rows = client.call("getGo2NearbyObstacles", query_radius, vehicle)
            model, data = obstacles.update(model, data, rows, home_position)
            if args.debug_obstacles and active_ids != obstacles.slots.keys():
                active_ids = set(obstacles.slots)
                print(f"{vehicle} UE collision boxes active: {sorted(active_ids)}", flush=True)

        t = i * dt
        ramp = max(0.0, min(1.0, (t - SETTLE) / RAMP))
        # Ease the diagonal trot in with zero slope at both ends.
        amp = ramp * ramp * (3.0 - 2.0 * ramp)

        phase = 2 * math.pi * FREQ * (t - SETTLE) + LEG_PHASE + START_PHASE
        delta = np.zeros(12)
        delta[1::3] = A_THIGH * np.sin(phase[1::3])
        delta[2::3] = A_CALF * np.sin(phase[2::3])
        if maneuver == "turn-walk" and t < TURN_DURATION + TURN_BLEND:
            # Reverse the left stride and blend it back after the pivot.
            blend = max(0.0, min(1.0, (t - TURN_DURATION) / TURN_BLEND))
            blend = blend * blend * (3.0 - 2.0 * blend)
            delta[list(LEFT_THIGH_INDICES)] *= 2.0 * blend - 1.0
        q_des = HOME + amp * delta
        if t < SETTLE:
            blend = t / SETTLE
            q_des = starting_angles + (HOME - starting_angles) * (blend * blend * (3.0 - 2.0 * blend))

        q = data.qpos[7:19]
        qd = data.qvel[6:18]
        data.ctrl[:] = np.clip(KP * (q_des - q) - KD * qd, ctrl_lo, ctrl_hi)
        mujoco.mj_step(model, data)
        if args.debug_obstacles and obstacles is not None:
            contacts = obstacles.contact_ids(model, data)
            if contacts - contact_ids:
                print(f"{vehicle} MuJoCo obstacle contact: {sorted(contacts - contact_ids)}", flush=True)
            contact_ids = contacts

        if i % send_every == 0 or i == steps - 1:
            feedback = client.call("moveGo2State", data.qpos[7:19].tolist(),
                                   (data.qpos[:3] - home_position).tolist(), data.qpos[3:7].tolist(), vehicle)
            if feedback:
                apply_body_feedback(model, data, home_position, feedback)
                if args.debug_obstacles and t - last_body_hit > 0.25:
                    print(f"{vehicle} UE body hit at {np.round(feedback[0], 3).tolist()}", flush=True)
                last_body_hit = t
            sent += 1

        target = wall_start + (i + 1) * dt
        remaining = target - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)

        if i % int(2.0 / dt) == 0:
            print(f"{vehicle}: t={t:4.1f}s  x={data.qpos[0]:6.2f}  z={data.qpos[2]:5.2f}", flush=True)

    print(f"{vehicle}: done, {args.duration:.1f}s sim, {sent} RPC updates", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        raise SystemExit(f"Go2 bridge failed: {exc}") from exc

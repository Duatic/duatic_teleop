#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy.serialization
from sensor_msgs.msg import JointState

from duatic_kinematics.pyroki_solver import PyrokiIKSolver

import subprocess

import math


# ============================================================
# CONFIG
# ============================================================

BAG_PATH = "/ros2_ws/bags/rosbag2_2026_04_02-11_15_33"  # folder, NOT file
JOINT_STATE_TOPIC = "/joint_states"

VEL_THRESHOLD = 2.0
ACC_THRESHOLD = 10.0
MANIP_THRESHOLD = 0.01


# Convert xacro to urdf because we use xacro files
def load_urdf_from_xacro(xacro_path):
    result = subprocess.run(
        ["xacro", xacro_path],
        capture_output=True,
        text=True,
        check=True
    )
    return result.stdout


# ============================================================
# LOAD BAG (MCAP)
# ============================================================

def load_joint_states_mcap(bag_path):
    storage_options = StorageOptions(
        uri=bag_path,
        storage_id="mcap"
    )

    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    joint_states = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == JOINT_STATE_TOPIC:
            msg = rclpy.serialization.deserialize_message(data, JointState)
            joint_states.append((timestamp, msg))

    return joint_states


# ============================================================
# ANALYSIS
# ============================================================

def compute_metrics(rows, solver):
    times = []
    velocities = []
    accelerations = []
    jerks = []
    manipulability = []

    last_vel = None
    last_acc = None
    last_time = None

    t0 = rows[0][0] * 1e-9

    total = len(rows)
    next_print = 1

    for i, (ts, msg) in enumerate(rows):
        progress = int((i / total) * 100)
        if progress >= next_print:
            print("\n---------------------------------")
            print(f"{progress}%")
            print("---------------------------------")
            next_print += 1

        q = np.array(msg.position)
        dq = np.array(msg.velocity)

        t = ts * 1e-9 - t0 # ns -> s and starting from rosbag start
        times.append(t)

        vel_norm = np.linalg.norm(dq)
        velocities.append(vel_norm)

        # dt-aware acceleration
        if last_vel is not None and last_time is not None:
            dt = t - last_time
            if dt > 0:
                acc = (dq - last_vel) / dt
                acc_norm = np.linalg.norm(acc)
            else:
                acc_norm = 0.0
        else:
            acc_norm = 0.0

        accelerations.append(acc_norm)

        # jerk
        if last_acc is not None and last_time is not None:
            dt = t - last_time
            if dt > 0:
                jerk = (acc_norm - last_acc) / dt
            else:
                jerk = 0.0
        else:
            jerk = 0.0

        jerks.append(abs(jerk))

        last_vel = dq
        last_acc = acc_norm
        last_time = t

        # ---------------- Jacobian ----------------
        try:
            J = numerical_jacobian(solver, q)
            JJ = J @ J.T
            w = np.sqrt(np.linalg.det(JJ))
        except Exception as e:
            print("Jacobian failed:", e)
            w = np.nan


        manipulability.append(w)

    return {
        "time": np.array(times),
        "vel": np.array(velocities),
        "acc": np.array(accelerations),
        "jerk": np.array(jerks),
        "manip": np.array(manipulability),
    }

def numerical_jacobian(solver, q, link_name=None, eps=1e-6):
    robot = solver.robot

    transforms = robot.forward_kinematics(q)

    # default: last link (flange)
    if link_name is None:
        idx = -1
    else:
        idx = robot.links.names.index(link_name)

    base = transforms[idx]
    pos0 = np.array(base[4:])  # xyz

    n = len(q)
    J = np.zeros((3, n))

    for i in range(n):
        q_eps = q.copy()
        q_eps[i] += eps

        transforms_eps = robot.forward_kinematics(q_eps)
        pos_eps = np.array(transforms_eps[idx][4:])

        J[:, i] = (pos_eps - pos0) / eps

    return J


# ============================================================
# DETECTION
# ============================================================

def detect_singularities(data):
    singular_events = []

    for i in range(len(data["time"])):

        if (
            data["manip"][i] < MANIP_THRESHOLD and
            data["vel"][i] > 0.2   # <-- KEY FIX
        ):
            singular_events.append({
                "time": data["time"][i],
                "manip": data["manip"][i],
                "vel": data["vel"][i],
                "acc": data["acc"][i],
            })

    return singular_events

def cluster_events(events, min_gap=2.0):
    """
    Groups events that are closer than min_gap seconds.
    Returns one representative per cluster.
    """

    if not events:
        return []

    # sort by time
    events_sorted = sorted(events, key=lambda e: e["time"])

    clustered = []
    current_cluster = [events_sorted[0]]

    for e in events_sorted[1:]:
        if e["time"] - current_cluster[-1]["time"] < min_gap:
            current_cluster.append(e)
        else:
            # pick worst in cluster (lowest manipulability)
            best = min(current_cluster, key=lambda x: x["manip"])
            clustered.append(best)
            current_cluster = [e]

    # last cluster
    best = min(current_cluster, key=lambda x: x["manip"])
    clustered.append(best)

    return clustered


# ============================================================
# VISUALIZATION
# ============================================================

def plot_results(data):
    t = data["time"]

    plt.figure(figsize=(12, 8))

    plt.subplot(4, 1, 1)
    plt.plot(t, data["vel"])
    plt.title("Joint Velocity Norm")

    plt.subplot(4, 1, 2)
    plt.plot(t, data["acc"])
    plt.title("Acceleration")

    plt.subplot(4, 1, 3)
    plt.plot(t, data["jerk"])
    plt.title("Jerk")

    plt.subplot(4, 1, 4)
    plt.plot(t, data["manip"])
    plt.title("Manipulability")

    plt.tight_layout()
    plt.show()


# ============================================================
# MAIN
# ============================================================

def main():
    print(f"Loading bag: {BAG_PATH}")

    rows = load_joint_states_mcap(BAG_PATH)
    print(f"Loaded {len(rows)} joint states")

    urdf_string = load_urdf_from_xacro("/home/benno/duatic/dev_workspace/src/duatic_dynaarm_demo/duatic_dynaarm_single_example/duatic_dynaarm_single_example_description/urdf/dynaarm_single_example.urdf.xacro")
    solver = PyrokiIKSolver(urdf_string)

    data = compute_metrics(rows, solver)

    events = detect_singularities(data)

    print(f"\nDetected {len(events)} potential singular events:\n")
    
    events_filtered = [e for e in events if e["vel"] > 0.5]
    
    print(f"\nDetected {len(events_filtered)} filtered singular events:\n")

    events_clustered = cluster_events(events_filtered, min_gap=3.0)

    print(f"\nReduced to {len(events_clustered)} independent events\n")

    events_sorted = sorted(events_clustered, key=lambda e: e["manip"])

    for e in events_sorted[:20]:
        print({
            "time": round(e["time"], 2),
            "manip": round(e["manip"], 5),
            "vel": round(e["vel"], 2),
            "acc": round(e["acc"], 2),
        })

    plot_results(data)


if __name__ == "__main__":
    main()
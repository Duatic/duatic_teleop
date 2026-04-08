#!/usr/bin/env python3

import numpy as np
import json
from scipy.spatial.transform import Rotation as R

def generate_targets(workspace, num_targets=200, seed=42):
    np.random.seed(seed)

    targets = []

    # ---------- 1. Define key extreme points ----------
    extremes = [
        [workspace["x"][0], 0.0, 0.5],
        [workspace["x"][1], 0.0, 0.5],
        [0.0, workspace["y"][0], 0.5],
        [0.0, workspace["y"][1], 0.5],
        [0.0, 0.0, workspace["z"][0]],
        [0.0, 0.0, workspace["z"][1]],
        [workspace["x"][1], workspace["y"][1], workspace["z"][1]],
        [workspace["x"][0], workspace["y"][0], workspace["z"][1]],
    ]

    # ---------- 2. Start from random point ----------
    pos = np.array([
        np.random.uniform(*workspace["x"]),
        np.random.uniform(*workspace["y"]),
        np.random.uniform(*workspace["z"]),
    ])

    for i in range(num_targets):

        # ---------- 3. Mix strategies ----------
        if i % 10 == 0:
            # jump to extreme
            new_pos = np.array(extremes[np.random.randint(len(extremes))])

        elif i % 10 == 1:
            # near boundary (singularity-prone)
            new_pos = np.array([
                np.random.choice([workspace["x"][0], workspace["x"][1]]),
                np.random.uniform(*workspace["y"]),
                np.random.uniform(*workspace["z"]),
            ])

        elif i % 10 == 2:
            # vertical sweep
            new_pos = np.array([
                pos[0],
                pos[1],
                np.random.uniform(*workspace["z"]),
            ])

        else:
            # large random step
            direction = np.random.randn(3)
            direction /= np.linalg.norm(direction)

            step_size = np.random.uniform(0.5, 1.5)
            new_pos = pos + direction * step_size

            # reflect at boundaries
            for j, axis in enumerate(["x", "y", "z"]):
                low, high = workspace[axis]
                if new_pos[j] < low:
                    new_pos[j] = low + (low - new_pos[j])
                if new_pos[j] > high:
                    new_pos[j] = high - (new_pos[j] - high)

        # ---------- 4. Orientation ----------
        # mix structured + random orientations
        if i % 4 == 0:
            quat = R.from_euler('xyz', [0, 0, 0]).as_quat()
        elif i % 4 == 1:
            quat = R.from_euler('xyz', [np.pi/2, 0, 0]).as_quat()
        elif i % 4 == 2:
            quat = R.from_euler('xyz', [0, np.pi/2, 0]).as_quat()
        else:
            quat = R.random().as_quat()

        quat_wxyz = [quat[3], quat[0], quat[1], quat[2]]

        targets.append({
            "position": new_pos.tolist(),
            "orientation": quat_wxyz
        })

        pos = new_pos

    return targets


def save_targets_to_json(targets, filename="robot_targets.json"):
    with open(filename, "w") as f:
        json.dump(targets, f, indent=2)


if __name__ == "__main__":
    workspace = {
        "x": [-1.0, 1.0],
        "y": [-1.0, 1.0],
        "z": [0.0, 1.5],
    }

    targets = generate_targets(workspace, num_targets=200)
    save_targets_to_json(targets)

    print("Saved 200 robot poses to robot_targets.json")
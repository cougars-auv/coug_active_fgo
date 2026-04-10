#!/usr/bin/env python3
# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import seaborn as sns
import scienceplots  # noqa: F401
from rosbags.highlevel import AnyReader
import yaml

sns.reset_orig()
plt.style.use(["science", "ieee"])

AGENTS = ["coug0sim", "coug1sim", "coug2sim", "blue0sim", "bluerov2"]
COLORS = {"Belief State MPPI": "#4C72B0", "Standard MPPI": "#DD8452"}


def is_critic_enabled(bag_path: Path) -> bool:
    config_path = bag_path / "config" / "fleet" / "coug_active_fgo_params.yaml"
    if not config_path.exists():
        return False
    with open(config_path) as f:
        config = yaml.safe_load(f)
    try:
        controller = config["/**"]["controller_server"]["ros__parameters"]["FollowPath"]
        return controller.get("BeliefStateCritic", {}).get("enabled", False)
    except (KeyError, TypeError):
        return False


def load_trace(bag_path: Path, agent: str) -> tuple[list[float], list[float]] | None:
    topic = f"/{agent}/belief_state_monitor_node/norm_trace"
    try:
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic == topic]
            if not connections:
                return None
            times, values = [], []
            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                msg = reader.deserialize(rawdata, connection.msgtype)
                times.append(timestamp * 1e-9)
                values.append(float(msg.data))  # type: ignore[union-attr]
            if not times:
                return None
            t0 = times[0]
            return [t - t0 for t in times], values
    except Exception as e:
        print(f"Error reading {bag_path}: {e}")
        return None


def plot_traces(target_dir: Path, output_path: Path) -> None:
    bag_paths = [p.parent for p in target_dir.rglob("metadata.yaml")]

    fig, ax = plt.subplots()
    active_plotted, baseline_plotted = False, False

    for bag_path in sorted(bag_paths):
        enabled = is_critic_enabled(bag_path)
        label_key = "Belief State MPPI" if enabled else "Standard MPPI"
        color = COLORS[label_key]

        for agent in AGENTS:
            result = load_trace(bag_path, agent)
            if result is None:
                continue
            times, values = result

            label = None
            if enabled and not active_plotted:
                label = label_key
                active_plotted = True
            elif not enabled and not baseline_plotted:
                label = label_key
                baseline_plotted = True

            ax.plot(times, values, color=color, linestyle="-", alpha=0.8, label=label)

    ax.set(title="", xlabel="Time (s)", ylabel="Normalized Bias Trace")
    if active_plotted or baseline_plotted:
        ax.legend()

    fig.savefig(str(output_path), dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: trace_plot.py <target_dir>")
        return

    target_dir = Path(sys.argv[1])
    if not target_dir.exists():
        print(f"Error: {target_dir} does not exist.")
        return

    plot_traces(target_dir, target_dir / "norm_trace.png")
    print(f"Plots saved to {target_dir}")


if __name__ == "__main__":
    main()

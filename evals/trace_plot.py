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
import scienceplots  # noqa: F401
import seaborn as sns
import yaml
from rosbags.highlevel import AnyReader

sns.reset_orig()
plt.style.use(["science", "ieee"])

AGENTS = ["coug1sim", "coug2sim", "coug3sim", "blue1sim", "bluerov2"]
COLORS = {"Belief State MPPI": "#4C72B0", "Standard MPPI": "#DD8452"}


def is_critic_enabled(bag_path: Path, agent_name: str) -> bool:
    """
    Read whether the belief state critic was enabled from a bag's config.

    :param bag_path: Path to the ROS 2 bag directory.
    :param agent_name: Agent namespace used to select namespaced parameters.
    :return: True if the belief state critic was enabled.
    """
    config_paths = [
        bag_path / "config" / "fleet" / "coug_active_fgo_params.yaml",
        bag_path / "config" / f"{agent_name}_params.yaml",
    ]

    enabled = False
    for path in config_paths:
        try:
            with open(path) as f:
                config = yaml.safe_load(f)
            try:
                enabled = config["/**"]["controller_server"]["ros__parameters"][
                    "FollowPath"
                ]["BeliefStateCritic"]["enabled"]
            except (KeyError, TypeError):
                pass
            try:
                enabled = config[f"/{agent_name}"]["controller_server"][
                    "ros__parameters"
                ]["FollowPath"]["BeliefStateCritic"]["enabled"]
            except (KeyError, TypeError):
                pass
        except (OSError, yaml.YAMLError):
            continue

    return bool(enabled)


def read_norm_trace(
    bag_path: Path, agent: str
) -> tuple[list[float], list[float]] | None:
    """
    Read the normalized bias trace time series for one agent from a bag.

    :param bag_path: Path to the ROS 2 bag directory.
    :param agent: Agent namespace to read the norm_trace topic for.
    :return: Zeroed timestamps and trace values, or None if unavailable.
    """
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


def render(target_dir: Path) -> None:
    """
    Save an overlaid normalized bias trace plot across all bags.

    :param target_dir: Directory tree containing the bags to plot.
    """
    bag_paths = [p.parent for p in target_dir.rglob("metadata.yaml")]

    fig, ax = plt.subplots()
    plotted_labels = set()

    for bag_path in sorted(bag_paths):
        for agent in AGENTS:
            if not (result := read_norm_trace(bag_path, agent)):
                continue
            times, values = result

            enabled = is_critic_enabled(bag_path, agent)
            label_key = "Belief State MPPI" if enabled else "Standard MPPI"

            label = None
            if label_key not in plotted_labels:
                label = label_key
                plotted_labels.add(label_key)

            ax.plot(
                times,
                values,
                color=COLORS[label_key],
                linestyle="-",
                alpha=0.8,
                label=label,
            )

    ax.set(title="", xlabel="Time (s)", ylabel="Normalized Bias Trace")
    if plotted_labels:
        ax.legend()

    fig.savefig(target_dir / "norm_trace.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: trace_plot.py <target_dir>")
        return

    target_dir = Path(sys.argv[1])
    if not target_dir.exists():
        print(f"Error: {target_dir} does not exist.")
        return

    render(target_dir)
    print(f"Plots saved to {target_dir}")


if __name__ == "__main__":
    main()

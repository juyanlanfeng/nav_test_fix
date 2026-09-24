"""Launch-time readiness gate shared by the three PB navigation entries.

The gate reuses pb_preflight's own checks instead of a fixed sleep: shortly
after the nodes are spawned it runs

    ros2 run pb_vehicle_adapter pb_preflight --framework <name> \
        --wait-timeout <startup_timeout_s>

which re-runs the real checks until they all pass or the timeout expires, then
prints the last report.  A timeout therefore names the missing conditions
(clock, TF, topics, nodes, actions, velocity source, map artifacts) instead of
starting navigation blindly.  The vehicle stays at zero velocity either way,
because the velocity adapter only forwards fresh commands from the selected
source.  `startup_timeout_s:=0` keeps the report but performs a single pass.
"""

from launch.actions import ExecuteProcess, TimerAction


# Time between spawning the nodes and the first check: a spawn delay, not the
# readiness mechanism itself.
SPAWN_GRACE_S = 2.0


def readiness_gate(framework, startup_timeout_s, period=SPAWN_GRACE_S):
    """Return the launch action that gates readiness for `framework`."""
    return TimerAction(
        period=period,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "pb_vehicle_adapter", "pb_preflight",
                    "--framework", framework,
                    "--wait-timeout", startup_timeout_s,
                ],
                output="screen",
            )
        ],
    )

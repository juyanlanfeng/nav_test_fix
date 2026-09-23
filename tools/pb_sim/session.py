#!/usr/bin/env python3
"""Run-session identity for PB simulation experiments (R5 of
doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md section 8).

One experiment = one immutable session file under ``log/pb_sim_sessions/``.  Every
terminal of that experiment loads the *same* file, so the ROS domain and the Gazebo
partition can never drift apart between terminals, and a new experiment cannot
silently change the identity of a running one.

This replaces the old ``log/.accept_run_id`` / ``log/.accept_domain`` pair, which:

* was a single global file that the next experiment overwrote, so a terminal that
  sourced the environment later joined the wrong experiment;
* was triggered by any non-empty ``ACCEPT_NEW_RUN`` including ``0``;
* was written non-atomically, so two shells starting together could interleave.
"""

import json
import os
from pathlib import Path
import random
import socket
import time

ROOT = Path(__file__).resolve().parents[2]
SESSION_DIR = ROOT / "log" / "pb_sim_sessions"
CYCLONEDDS_URI = "file://%s/log/cyclonedds.xml" % ROOT

# The URI above is only usable if the file it names exists: CycloneDDS refuses to
# create a domain when it cannot open the configuration, so every ROS node in a
# shell that exported the URI aborts with "can't open configuration file" and
# `rmw_create_node: failed to create domain`, including RViz and the spawner.  The
# file used to be assumed rather than created, which broke whole runs whenever
# log/ was cleaned.  It is now written on demand.
CYCLONEDDS_XML = """<?xml version="1.0" encoding="UTF-8"?>
<!-- Local discovery configuration handed out by tools/pb_sim/session.py.
     CycloneDDS reads this through CYCLONEDDS_URI=%s.
     If this file is missing, every ROS node in a shell that exported that URI
     fails with "can't open configuration file" and rmw_create_node aborts. -->
<CycloneDDS>
  <Domain Id="any">
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
""" % CYCLONEDDS_URI


def ensure_cyclonedds_config(path=None):
    """Write the DDS configuration named by CYCLONEDDS_URI if it is not there.

    Returns the path.  Called before a session is created and before the
    environment is emitted, so a shell can never be handed a URI whose target does
    not exist.  Existing content is left untouched: an operator may have tuned it
    deliberately.
    """
    target = Path(path) if path else Path(CYCLONEDDS_URI[len("file://"):])
    if not target.is_file():
        write_atomic(target, CYCLONEDDS_XML)
    return target
ROS_HOME = str(ROOT / "log" / "ros_accept_home")

TRUTHY = {"1", "true", "yes", "on"}
FALSY = {"0", "false", "no", "off", ""}


def parse_bool(value, name="value"):
    """Strict boolean parsing: anything else is an error, not a truthy string."""
    if isinstance(value, bool):
        return value
    if value is None:
        raise ValueError("%s must be one of %s / %s" % (name, sorted(TRUTHY), sorted(FALSY)))
    text = str(value).strip().lower()
    if text in TRUTHY:
        return True
    if text in FALSY:
        return False
    raise ValueError("%s must be one of %s / %s, got %r"
                     % (name, sorted(TRUTHY), sorted(FALSY), value))


def format_sim_time(sec, nanosec):
    """Format a simulation time as seconds with exactly nine nanosecond digits.

    The old harness formatted ``sec.nanosec`` directly, so sec=1, nanosec=2 was
    recorded as 1.2 s instead of 1.000000002 s.
    """
    return "%d.%09d" % (int(sec), int(nanosec))


def format_stamp(message):
    """Format a ROS message header stamp (builtin_interfaces/Time)."""
    return format_sim_time(message.header.stamp.sec, message.header.stamp.nanosec)


def new_session_id(label="run"):
    suffix = "%04x" % random.SystemRandom().randrange(0x10000)
    return "%s-%s-%d-%s" % (label, time.strftime("%Y%m%d-%H%M%S", time.gmtime()),
                            os.getpid(), suffix)


def create_session(domain, label="run", partition=None, session_dir=None):
    """Create a new immutable session file and return its record.

    Raises if the generated id already exists (astronomically unlikely, but the
    uniqueness is checked rather than assumed).
    """
    directory = Path(session_dir) if session_dir else SESSION_DIR
    directory.mkdir(parents=True, exist_ok=True)
    ensure_cyclonedds_config()
    domain = int(domain)
    session_id = new_session_id(label)
    partition = partition or "pb_%s" % session_id
    record = {
        "session_id": session_id,
        "label": label,
        "domain": domain,
        "partition": partition,
        "created_wall": format_sim_time(int(time.time()), 0),
        "host": socket.gethostname(),
        "ros_home": ROS_HOME,
        "cyclonedds_uri": CYCLONEDDS_URI,
    }
    path = directory / ("%s.json" % session_id)
    if path.exists():
        raise RuntimeError("session file already exists: %s" % path)
    write_atomic(path, json.dumps(record, indent=2, sort_keys=True) + "\n")
    return path, record


def write_atomic(path, text):
    """Write via a temp file in the same directory and rename it into place."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp-%d" % os.getpid())
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def load_session(path):
    """Load a session by explicit path.  There is deliberately no "latest" lookup."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError("no session file at %s" % path)
    record = json.loads(path.read_text(encoding="utf-8"))
    for key in ("session_id", "domain", "partition"):
        if key not in record:
            raise ValueError("session file %s is missing %r" % (path, key))
    return record


def session_environment(record):
    """The complete, mutually consistent environment of one session.

    Self-healing: the returned CYCLONEDDS_URI is only handed out once its target
    exists, so a cleaned log/ cannot produce an environment that kills every node.
    """
    uri = record.get("cyclonedds_uri", CYCLONEDDS_URI)
    if uri.startswith("file://"):
        ensure_cyclonedds_config(uri[len("file://"):])
    return {
        "ROS_DOMAIN_ID": str(record["domain"]),
        "ROS_LOCALHOST_ONLY": "1",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": uri,
        "IGN_PARTITION": record["partition"],
        "GZ_PARTITION": record["partition"],
        "ROS_HOME": record.get("ros_home", ROS_HOME),
        "PB_SIM_SESSION": str(record["session_id"]),
    }


def write_env_file(record, path):
    """Write a shell-sourceable file with every derived value updated together."""
    lines = ["# Generated from session %s; source it, do not edit it."
             % record["session_id"]]
    for key, value in sorted(session_environment(record).items()):
        lines.append("export %s=%s" % (key, shell_quote(value)))
    text = "\n".join(lines) + "\n"
    write_atomic(path, text)
    return Path(path)


def shell_quote(value):
    return "'" + str(value).replace("'", "'\\''") + "'"


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command", required=True)

    create = subparsers.add_parser("create", help="create a new session")
    create.add_argument("--domain", type=int, required=True)
    create.add_argument("--label", default="run")
    create.add_argument("--partition")
    create.add_argument("--session-dir")
    create.add_argument("--env-out", help="also write a sourceable environment file")

    show = subparsers.add_parser("env", help="print the session environment")
    show.add_argument("session")

    arguments = parser.parse_args()
    if arguments.command == "create":
        path, record = create_session(arguments.domain, arguments.label,
                                      arguments.partition, arguments.session_dir)
        if arguments.env_out:
            write_env_file(record, arguments.env_out)
        print(json.dumps({"session_file": str(path), **record}, indent=2, sort_keys=True))
        return 0
    record = load_session(arguments.session)
    for key, value in sorted(session_environment(record).items()):
        print("export %s=%s" % (key, shell_quote(value)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

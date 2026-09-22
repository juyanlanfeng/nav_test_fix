# tools/pb_sim — tracked PB simulation tools (R5)

These replace the unversioned helpers that used to live in `log/` (which is
git-ignored, so a clean checkout could not reproduce an experiment).  Runtime
artifacts (sessions, reports, bags) still go to `log/`; only the tools are tracked.

## One experiment = one session file

```bash
# pick a domain with no publisher (check first), then create the session
python3 tools/pb_sim/session.py create --domain 187 --label ramp \
  --env-out log/pb_sim_sessions/ramp.env
```

Every terminal of that experiment sources the *same* file, so the ROS domain and
the Gazebo partition cannot drift apart:

```bash
source log/pb_sim_sessions/ramp.env
```

Rules encoded here (plan section 8):

* `ACCEPT_NEW_RUN`-style values are parsed strictly — `"0"` is false, not "set";
* a session file is created with a unique id and an atomic rename, and is never
  overwritten, so a new experiment cannot change a running one;
* there is no implicit "latest session" lookup: consumers name the file;
* every derived value (`ROS_DOMAIN_ID`, `IGN_PARTITION`, `GZ_PARTITION`, DDS URI,
  `ROS_HOME`) is written together.

## Isolation check

```bash
python3 tools/pb_sim/check_isolation.py --report log/isolation.json
```

Requires exactly one owner for `/clock`, `/odom` and the command outlet
`/pb/cmd_vel_safe`, requires the expected TF owners, requires the Gazebo partition
to expose the robot command and world-pose topics, and treats an empty or
timed-out query as a failure.

## Collect one run

```bash
tools/pb_sim/collect_run.sh --session log/pb_sim_sessions/<id>.json \
  --framework meshnav --x 1.35 --y 5.95 --z 0.203 --yaw 0 \
  --spawn-x -1.90 --spawn-y 5.95 --cancel-after 30 --label ramp
```

The collector gates on the isolation check and on `pb_preflight` return codes,
waits for the recorder to show real `/clock` and `/odom` data before sending the
goal, cancels through R2's `--cancel-after` (same UUID), and tears down only its
own process trees with EXIT/INT/TERM traps — stopping the recorder first so the bag
is flushed.  Environment and asset hashes are captured after the domain and
partition are final.

## Analyze

```bash
python3 tools/pb_sim/analyze_report.py log/<label>_report_<stamp> \
  --goal-x 1.35 --goal-y 5.95
```

Reports, with no silent fallbacks: simulation vs wall duration from `/clock`;
per-UUID action sequences with terminal results; command frequency, gaps and
non-zero windows (bare `Twist` included, placed on the simulation timeline through
`/clock`); linear and angular magnitudes separately; XYZ/attitude ranges,
cumulative displacement, stop intervals; Gazebo message counts and RTF; and the
Gazebo-truth/`/odom` extrinsic consistency.  Missing or unparsable input is
reported as `missing` and fails the analysis.

## Verify the new truth source

```bash
python3 tools/pb_sim/verify_truth_source.py --seconds 6 --drive 0.2
python3 tools/pb_sim/verify_truth_source.py --seconds 3 --pause-test
```

## Tests

```bash
cd tools && python3 -m pytest pb_sim/test_pb_sim_tools.py -q
```

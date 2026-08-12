#!/usr/bin/env python3
"""Monte-Carlo sweep runner: a mission controller vs its SIL harness, many times.

For every parameter combination it runs N seeded episodes (driver + harness
as subprocesses on an isolated ROS_DOMAIN_ID), parses the harness RESULT
line, and appends one CSV row per episode. Source the workspaces first:

    source ~/eufs/install/setup.bash && source ~/autonomy_ws/install/setup.bash

Examples:

    # trackdrive detection-dropout sweep, 20 episodes per point, 3x realtime
    python3 scripts/run_sweeps.py --episodes 20 --speed 3 \
        --sweep p_detect_scale=1.0,0.8,0.6,0.4,0.2 --out dropout.csv

    # skidpad odometry-drift sweep (the state-estimation experiment)
    python3 scripts/run_sweeps.py --mission skidpad --episodes 20 --speed 3 \
        --sweep odom_yaw_drift_deg_per_sqrt_min=0,1,2,4,8 --out skid_drift.csv

    # measured YOLO profile, degraded progressively
    python3 scripts/run_sweeps.py --episodes 20 --speed 3 \
        --profile perception/error_profile.json \
        --sweep p_detect_scale=1.0,0.8,0.6 --out yolo_profile.csv

Unknown --sweep/--fixed params route to the harness; params named in the
mission's driver set route to the driver.
"""

import argparse
import csv
import itertools
import math
import os
import signal
import subprocess
import sys
import time

COMMON_DRIVER = {"mission", "odom_topic", "accel_limit", "brake_limit", "kp",
                 "stop_speed", "brake_hold", "target_speed"}


def accel_success(r, args):
    return (r["brake_x"] != "never" and r["stop_x"] != "never"
            and float(r["final_v"]) < 0.05)


def skidpad_success(r, args):
    return (abs(float(r["turn_total"]) - 8.0 * math.pi) < 0.6
            and float(r["circ_err_max"]) < 1.0
            and float(r["final_x"]) > 25.0
            and float(r["final_v"]) < 0.05)


def trackdrive_success(r, args):
    return (int(r["laps"]) >= args.laps
            and int(r["cones_hit"]) == 0
            and float(r["final_v"]) < 0.05)


MISSIONS = {
    "accel": {
        "driver": "accel_driver",
        "harness": "sil_accel",
        "driver_params": COMMON_DRIVER | {"finish_distance", "publish_rate"},
        "rate_param": "publish_rate",
        "metrics": ["max_v", "min_a", "brake_x", "stop_x", "final_x", "final_v"],
        "success": accel_success,
        "sim_duration": 40.0,
    },
    "skidpad": {
        "driver": "skidpad_driver",
        "harness": "sil_skidpad",
        "driver_params": COMMON_DRIVER | {
            "entry_distance", "circle_radius", "laps_per_circle",
            "exit_distance", "lookahead", "wheelbase", "steer_limit",
            "tick_rate"},
        "rate_param": "tick_rate",
        "metrics": ["max_v", "circ_err_mean", "circ_err_max", "turn_total",
                    "net_yaw", "final_x", "final_y", "final_v"],
        "success": skidpad_success,
        "sim_duration": 100.0,
    },
    "trackdrive": {
        "driver": "trackdrive_driver",
        "harness": "sil_trackdrive",
        "driver_params": COMMON_DRIVER | {
            "cones_topic", "laps", "min_speed", "max_cone_range",
            "min_target_dist", "half_track", "cone_timeout",
            "blind_stop_time", "lap_arm_distance", "lap_close_distance",
            "wheelbase", "steer_limit", "tick_rate"},
        "rate_param": "tick_rate",
        "metrics": ["laps", "dev_mean", "dev_max", "cones_hit",
                    "max_v", "time", "final_v"],
        "success": trackdrive_success,
        "sim_duration": 240.0,
    },
}

INT_PARAMS = {"laps", "seed", "latency_frames", "laps_per_circle"}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mission", choices=sorted(MISSIONS), default="trackdrive")
    ap.add_argument("--episodes", type=int, default=10, help="episodes per config")
    ap.add_argument("--sweep", action="append", default=[],
                    help="param=v1,v2,... (repeatable; repeats form a grid)")
    ap.add_argument("--fixed", action="append", default=[],
                    help="param=value applied to every episode (repeatable)")
    ap.add_argument("--profile", default="", help="error_profile.json for the harness")
    ap.add_argument("--laps", type=int, default=3,
                    help="trackdrive: laps per episode / pass bar")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="realtime multiple (harness realtime_factor + driver rate)")
    ap.add_argument("--domain", type=int, default=60, help="ROS_DOMAIN_ID to use")
    ap.add_argument("--seed-base", type=int, default=1000)
    ap.add_argument("--out", default="sweeps.csv")
    return ap.parse_args()


def fmt_value(key, val):
    """Format a value for ros2 -p so the declared parameter type matches."""
    if key in INT_PARAMS:
        return str(int(float(val)))
    try:
        return repr(float(val))  # always a decimal point -> float param
    except ValueError:
        return str(val)  # string param (mission, cones_topic, profile path)


def ros_args(params):
    out = ["--ros-args"]
    for k, v in params.items():
        out += ["-p", "%s:=%s" % (k, fmt_value(k, v))]
    return out


def spawn(cmd, env, capture=False):
    return subprocess.Popen(
        cmd, env=env, preexec_fn=os.setsid,
        stdout=subprocess.PIPE if capture else subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, text=True)


def stop(proc):
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def run_episode(mission, cfg, seed, args, env):
    driver_p = {mission["rate_param"]: 50.0 * args.speed}
    if args.mission == "trackdrive":
        driver_p["laps"] = args.laps
    harness_p = {"seed": seed, "realtime_factor": args.speed}
    if args.profile:
        harness_p["profile_json"] = os.path.abspath(args.profile)
    for k, v in cfg.items():
        (driver_p if k in mission["driver_params"] else harness_p)[k] = v

    driver = spawn(["ros2", "run", "fs_autonomy", mission["driver"]]
                   + ros_args(driver_p), env)
    time.sleep(2.0)  # let the driver come up and subscribe
    harness = spawn(["ros2", "run", "fs_autonomy", mission["harness"]]
                    + ros_args(harness_p), env, capture=True)

    result = None
    try:
        out, _ = harness.communicate(
            timeout=mission["sim_duration"] / args.speed + 60)
        for line in out.splitlines():
            if line.startswith("RESULT "):
                result = dict(tok.split("=") for tok in line.split()[1:])
    except subprocess.TimeoutExpired:
        pass
    finally:
        stop(harness)
        stop(driver)
    return result


def main():
    args = parse_args()
    mission = MISSIONS[args.mission]

    sweeps = []
    for s in args.sweep:
        key, vals = s.split("=", 1)
        sweeps.append([(key.strip(), v.strip()) for v in vals.split(",")])
    fixed = dict(f.split("=", 1) for f in args.fixed)
    configs = [dict(fixed, **dict(combo))
               for combo in (itertools.product(*sweeps) if sweeps else [()])]
    swept_keys = [s[0][0] for s in sweeps]

    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    cols = (sorted(set(k for c in configs for k in c)) + ["seed"]
            + mission["metrics"] + ["success"])

    total = len(configs) * args.episodes
    done = 0
    t0 = time.time()
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for cfg in configs:
            passes = 0
            for ep in range(args.episodes):
                seed = args.seed_base + ep
                r = run_episode(mission, cfg, seed, args, env)
                done += 1
                row = dict(cfg, seed=seed)
                if r is None:
                    row["success"] = 0  # crashed or hung = failure
                else:
                    for k in mission["metrics"]:
                        row[k] = r.get(k, "")
                    try:
                        row["success"] = int(mission["success"](r, args))
                    except (KeyError, ValueError):
                        row["success"] = 0
                    passes += row["success"]
                w.writerow(row)
                f.flush()
                label = ", ".join("%s=%s" % (k, cfg[k]) for k in swept_keys) or "baseline"
                print("[%d/%d] %s %s seed=%d -> %s   (%.0fs elapsed)"
                      % (done, total, args.mission, label, seed,
                         "PASS" if row["success"] else "FAIL", time.time() - t0),
                      flush=True)
            print(">> %s %s: %d/%d pass" % (args.mission, label, passes,
                                            args.episodes), flush=True)

    print("wrote", args.out)


if __name__ == "__main__":
    main()

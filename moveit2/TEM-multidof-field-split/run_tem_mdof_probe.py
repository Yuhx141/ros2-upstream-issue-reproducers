#!/usr/bin/env python3
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent
BUILD = Path(os.environ.get("P3_BUILD_DIR", "/tmp/p3-moveit-tem-p0-build"))
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results"
MODES = ("single_dof", "mdof_one", "mdof_two_reorder")
RUNS = 3

def run(command, env=None):
    return subprocess.run(command, text=True, capture_output=True, env=env, check=False)

def values(text):
    return [] if text == "" else [float(value) for value in text.split(",")]

OUT.mkdir(parents=True, exist_ok=True)
configure = run(["cmake", "-S", str(ROOT), "-B", str(BUILD)])
(OUT / "build-configure.log").write_text(configure.stdout + configure.stderr)
if configure.returncode:
    raise SystemExit(configure.returncode)
build = run(["cmake", "--build", str(BUILD), "-j2"])
(OUT / "build.log").write_text(build.stdout + build.stderr)
if build.returncode:
    raise SystemExit(build.returncode)

cases = []
for mode_index, mode in enumerate(MODES):
    for repeat in range(RUNS):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(180 + mode_index * RUNS + repeat)
        result = run([str(BUILD / "tem_mdof_probe"), mode], env=env)
        log = result.stdout + result.stderr
        name = f"{mode}-{repeat + 1}"
        (OUT / f"{name}.log").write_text(log)
        match = re.search(r"P3_RESULT (\{.*\})", log)
        observed = json.loads(match.group(1)) if match else {}
        basic = (
            result.returncode == 0
            and observed.get("harness_valid") is True
            and observed.get("push_return") is True
            and not observed.get("exception")
            and observed.get("queued") == 1
        )
        if mode == "single_dof":
            oracle = observed.get("out_joint_names") == "slide"
        elif mode == "mdof_one":
            oracle = (
                observed.get("out_mdof_names") == "base_joint"
                and values(observed.get("out_transform_x", "")) == [10.0]
                and values(observed.get("out_velocity_x", "")) == [1.0]
                and values(observed.get("out_acceleration_x", "")) == []
            )
        else:
            oracle = (
                observed.get("out_mdof_names") == "base_joint,tool_joint"
                and values(observed.get("out_transform_x", "")) == [10.0, 20.0]
                and values(observed.get("out_velocity_x", "")) == [1.0, 2.0]
                and values(observed.get("out_acceleration_x", "")) == [3.0, 4.0]
            )
        passed = basic and oracle
        cases.append({
            "case": name,
            "mode": mode,
            "repeat": repeat + 1,
            "status": "pass" if passed else ("violation" if basic else "execution_error"),
            "returncode": result.returncode,
            "oracle": oracle,
            "observed": observed,
            "log": f"{name}.log",
        })

manifest = {
    "schema": "p3-moveit-tem-mdof-split-v1",
    "created_at": datetime.now(timezone.utc).isoformat(),
    "versions": {"moveit_ros_planning": "2.12.4", "ros_distro": "jazzy"},
    "runs_per_arm": RUNS,
    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "probe_sha256": hashlib.sha256((ROOT / "tem_mdof_probe.cpp").read_bytes()).hexdigest(),
    "cases": cases,
}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
summary = {status: sum(c["status"] == status for c in cases) for status in ("pass", "violation", "execution_error")}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
print(json.dumps(summary, sort_keys=True))

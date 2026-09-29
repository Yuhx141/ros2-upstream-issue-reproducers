#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path


def write(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    arms = ("always_success", "requires_params", "always_fail")
    manifest = {
        "schema": "p3-moveit-occupancy-init-v1",
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "repetitions": args.repetitions,
        "arms": arms,
        "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    write(args.output / "manifest.json", manifest)
    records = []
    for repeat in range(1, args.repetitions + 1):
        for arm in arms:
            process = subprocess.run([str(args.executable), arm], text=True, capture_output=True, timeout=10,
                                     env=dict(os.environ, ROS_DOMAIN_ID=str(210 + repeat)))
            lines = [x.removeprefix("P3_RESULT ") for x in process.stdout.splitlines()
                     if x.startswith("P3_RESULT ")]
            if process.returncode or len(lines) != 1:
                record = {"repeat": repeat, "arm": arm, "classification": "execution_error",
                          "returncode": process.returncode, "stdout": process.stdout, "stderr": process.stderr}
            else:
                actual = json.loads(lines[0])
                if arm == "always_success":
                    checks = {"success_control_starts": actual["init_result"] and actual["started"]}
                elif arm == "requires_params":
                    checks = {
                        "params_precede_initialize": actual["calls"][:2] == ["set_params", "initialize"],
                        "configured_initialize_succeeds": actual["init_result"],
                        "successful_updater_starts": actual["started"],
                    }
                else:
                    checks = {"failed_initialize_does_not_start": not actual["init_result"] and not actual["started"]}
                passed = all(checks.values())
                record = {"repeat": repeat, "arm": arm, "classification": "pass" if passed else "violation",
                          "controls_pass": actual["params_set"], "checks": checks, "actual": actual,
                          "stdout": process.stdout, "stderr": process.stderr}
            records.append(record)
            write(args.output / f"repeat-{repeat:02d}-{arm}.json", record)
            print(json.dumps({"repeat": repeat, "arm": arm, "classification": record["classification"]}), flush=True)
    counts = {key: sum(x["classification"] == key for x in records)
              for key in ("pass", "violation", "execution_error")}
    counts["total"] = len(records)
    write(args.output / "summary.json", {"schema": manifest["schema"], "counts": counts, "records": records,
                                          "completed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
    print(json.dumps(counts, sort_keys=True))
    return 1 if counts["execution_error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

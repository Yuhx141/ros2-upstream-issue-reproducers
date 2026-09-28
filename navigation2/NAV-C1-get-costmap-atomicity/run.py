#!/usr/bin/env python3
"""Run the deterministic GetCostmap atomicity probe with a resumable ledger."""

import argparse
import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def write_json(path, value):
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def summarize(root, manifest, results):
    counts = {name: sum(item.get("classification") == name for item in results)
              for name in ("partial_response", "atomic_response", "unexpected")}
    summary = {
        "manifest": manifest,
        "completed": len(results),
        "execution_errors": sum("execution_error" in item for item in results),
        "classifications": counts,
        "results": results,
    }
    write_json(root / "summary.json", summary)
    (root / "summary.md").write_text(
        "# Deterministic GetCostmap atomicity probe\n\n"
        f"- completed: {len(results)} / {manifest['repetitions']}\n"
        f"- partial responses: {counts['partial_response']}\n"
        f"- atomic responses: {counts['atomic_response']}\n"
        f"- unexpected: {counts['unexpected']}\n"
        f"- execution errors: {summary['execution_errors']}\n",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("repetitions must be positive")
    binary, root = args.binary.resolve(), args.out_dir.resolve()
    if not binary.is_file():
        parser.error(f"probe binary is missing: {binary}")
    script = Path(__file__).resolve()
    identity = {
        "repetitions": args.repetitions,
        "binary": str(binary),
        "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
    }
    if root.exists():
        if not args.resume:
            parser.error("out-dir exists; pass --resume to continue it")
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        if any(manifest[key] != value for key, value in identity.items()):
            parser.error("resume manifest does not match repetitions, runner, or binary")
    else:
        root.mkdir(parents=True)
        manifest = {"schema": 1, "started_at": datetime.now(timezone.utc).isoformat(),
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"), **identity}
        write_json(root / "manifest.json", manifest)
    ledger = root / "results.jsonl"
    results = ([json.loads(line) for line in ledger.read_text(encoding="utf-8").splitlines()
                if line.strip()] if ledger.exists() else [])
    if len(results) > args.repetitions:
        parser.error("result ledger is longer than requested repetitions")
    for index in range(len(results), args.repetitions):
        completed = subprocess.run([str(binary)], text=True, capture_output=True, timeout=8)
        try:
            observation = json.loads(completed.stdout.strip().splitlines()[-1])
            if completed.returncode and observation.get("classification") != "unexpected":
                raise ValueError("nonzero exit for a recognized classification")
            observation["repeat"] = index
        except Exception as error:
            observation = {"repeat": index, "execution_error": repr(error),
                           "returncode": completed.returncode,
                           "stdout": completed.stdout[-1000:], "stderr": completed.stderr[-1000:]}
        with ledger.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(observation, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        results.append(observation)
        summarize(root, manifest, results)
        print(json.dumps(observation, sort_keys=True), flush=True)
    summarize(root, manifest, results)
    raise SystemExit(1 if any("execution_error" in item or
                              item.get("classification") == "unexpected" for item in results) else 0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import pathlib
import subprocess

CASES = (
    ("enabled_limit_controls", "JointTransmissionP0.EnabledLimitControls", True),
    ("disabled_position_keeps_velocity_limit",
     "JointTransmissionP0.DisabledPositionKeepsVelocityLimit", True),
    ("single_dynamic_update", "JointTransmissionP0.SingleDynamicUpdate", True),
    ("simple_formula_power", "JointTransmissionP0.SimpleFormulaPower", True),
    ("differential_formula_power", "JointTransmissionP0.DifferentialFormulaPower", True),
    ("four_bar_formula_power", "JointTransmissionP0.FourBarFormulaPower", True),
    ("atomic_change_then_same", "JointTransmissionP0.AtomicChangeThenSame", False),
    ("disable_position_runtime", "JointTransmissionP0.DisablePositionRuntime", False),
    ("disabled_position_retained_range", "JointTransmissionP0.DisabledPositionRetainedRange", False),
    ("disabled_position_retained_soft", "JointTransmissionP0.DisabledPositionRetainedSoft", False),
    ("disabled_jerk_retained_range", "JointTransmissionP0.DisabledJerkRetainedRange", False),
    ("disabled_jerk_retained_soft", "JointTransmissionP0.DisabledJerkRetainedSoft", False),
)


def run_one(output, binary, name, test, repetition):
    case_dir = output / name / f"run-{repetition}"
    case_dir.mkdir(parents=True)
    gtest_json = case_dir / "gtest.json"
    command = [str(binary), f"--gtest_filter={test}", f"--gtest_output=json:{gtest_json}"]
    try:
        done = subprocess.run(
            command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=20, check=False)
        returncode, log, timed_out = done.returncode, done.stdout, False
    except subprocess.TimeoutExpired as error:
        returncode = None
        raw = error.stdout or ""
        log = raw.decode(errors="replace") if isinstance(raw, bytes) else raw
        log += "\nP3_RUNNER timeout=20s\n"
        timed_out = True
    (case_dir / "output.log").write_text(log, encoding="utf-8", errors="replace")
    return {
        "schema": "p3-joint-transmission-p0-result-v1", "case": name, "test": test,
        "repetition": repetition, "command": command, "returncode": returncode,
        "signal": -returncode if returncode is not None and returncode < 0 else None,
        "timed_out": timed_out, "test_started": f"[ RUN      ] {test}" in log,
        "observations": [line for line in log.splitlines() if line.startswith("P3_OBSERVATION")],
        "gtest_json": str(gtest_json) if gtest_json.exists() else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("binary")
    parser.add_argument("--repetitions", type=int, default=3)
    args = parser.parse_args()
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    binary = pathlib.Path(args.binary).resolve()
    if not binary.is_file():
        raise SystemExit("test binary is missing")
    records = []
    for name, test, control in CASES:
        if control:
            for repetition in range(1, args.repetitions + 1):
                records.append(run_one(output, binary, name, test, repetition))
    apparatus_ok = all(
        record["test_started"] and record["returncode"] == 0 and record["observations"]
        and not record["timed_out"] for record in records)
    for name, test, control in CASES:
        if not control:
            for repetition in range(1, args.repetitions + 1):
                records.append(run_one(output, binary, name, test, repetition))
    controls = {name for name, _, control in CASES if control}
    for record in records:
        valid = record["test_started"] and not record["timed_out"] and record["observations"]
        if record["case"] in controls:
            record["verdict"] = "pass" if valid and record["returncode"] == 0 else "execution_error"
        elif not apparatus_ok or not valid:
            record["verdict"] = "execution_error"
        else:
            record["verdict"] = "pass" if record["returncode"] == 0 else "violation"
    keys = ("pass", "violation", "execution_error")
    summary = {
        "schema": "p3-joint-transmission-p0-summary-v1",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "binary": str(binary), "repetitions": args.repetitions,
        "apparatus_ok": apparatus_ok,
        "counts": {key: sum(record["verdict"] == key for record in records) for key in keys},
        "case_counts": {
            name: {key: sum(record["case"] == name and record["verdict"] == key
                            for record in records) for key in keys}
            for name, _, _ in CASES},
    }
    with (output / "results.jsonl").open("w", encoding="utf-8") as result_file:
        for record in records:
            result_file.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    raise SystemExit(0 if not summary["counts"]["execution_error"] else 2)


if __name__ == "__main__":
    main()

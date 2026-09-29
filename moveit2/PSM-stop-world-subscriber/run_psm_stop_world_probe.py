#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def version(package):
    return subprocess.check_output(['ros2', 'pkg', 'xml', '-t', 'version', package], text=True).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--domain-base', type=int, default=180)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {
        'schema': 'p3-moveit-psm-stop-world-v1',
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'repetitions': args.repetitions,
        'arms': ['single_stop', 'double_stop'],
        'executable': str(args.executable.resolve()),
        'source': str(args.source.resolve()),
        'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'packages': {'moveit_ros_planning': version('moveit_ros_planning')},
    }
    write(args.output / 'manifest.json', manifest)
    records = []
    for repetition in range(args.repetitions):
      for offset, arm in enumerate(manifest['arms']):
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_base + repetition * 2 + offset))
        if arm == 'double_stop':
            env['P3_DOUBLE_STOP'] = '1'
        process = subprocess.run([str(args.executable)], env=env, text=True, capture_output=True, timeout=15)
        result_lines = [x.removeprefix('P3_RESULT ') for x in process.stdout.splitlines() if x.startswith('P3_RESULT ')]
        if process.returncode != 0 or len(result_lines) != 1:
            record = {'repetition': repetition, 'arm': arm, 'classification': 'execution_error', 'pass': False,
                      'returncode': process.returncode, 'stdout': process.stdout, 'stderr': process.stderr}
        else:
            actual = json.loads(result_lines[0])
            passed = (actual['collision_subscriptions_after'] == 0 and
                      actual['world_subscriptions_after'] == 0 and
                      not actual['collision_updated_after_stop'] and
                      not actual['world_updated_after_stop'] and
                      actual['topics_after'] == '')
            record = {'repetition': repetition, 'arm': arm, 'classification': 'pass' if passed else 'violation',
                      'pass': passed,
                      'expected': 'both world-geometry subscriptions removed and both inputs silent after stop',
                      'actual': actual, 'stdout': process.stdout, 'stderr': process.stderr}
        records.append(record)
        write(args.output / f'repeat-{repetition:02d}-{arm}.json', record)
        write(args.output / 'progress.json', {'completed': len(records), 'total': args.repetitions * len(manifest['arms']),
              'pass': sum(x['classification'] == 'pass' for x in records),
              'violation': sum(x['classification'] == 'violation' for x in records),
              'execution_error': sum(x['classification'] == 'execution_error' for x in records)})
        print(json.dumps({'repetition': repetition, 'arm': arm, 'classification': record['classification']}), flush=True)
    counts = {'total': len(records), 'pass': sum(x['classification'] == 'pass' for x in records),
              'violation': sum(x['classification'] == 'violation' for x in records),
              'execution_error': sum(x['classification'] == 'execution_error' for x in records)}
    write(args.output / 'summary.json', {'schema': manifest['schema'], 'counts': counts, 'records': records,
          'completed_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    print(json.dumps(counts))
    return 1 if counts['execution_error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())

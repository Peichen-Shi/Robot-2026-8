#!/usr/bin/env python3
"""Read-only ROS-free audit of existing bags; never starts a simulator.

Requires rosbags (pip install rosbags). Output is separate from original evidence.
"""
import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'optimization'))
from race_acceptance import evaluate_run


class PortableBag:
    def __init__(self, path):
        from rosbags.highlevel import AnyReader
        self.reader = AnyReader([Path(path)])

    def __enter__(self):
        self.reader.open()
        return self

    def __exit__(self, *args):
        self.reader.close()

    def read_messages(self, topics):
        connections = [c for c in self.reader.connections if c.topic in topics]
        for connection, timestamp, raw in self.reader.messages(connections=connections):
            message = self.reader.deserialize(raw, connection.msgtype)
            # rosbags represents ROS time using sec/nanosec fields.
            stamp = message.header.stamp
            seconds = stamp.sec + stamp.nanosec * 1e-9
            message.header.stamp = SimpleNamespace(to_sec=lambda t=seconds: t)
            yield connection.topic, message, timestamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = []
    for directory in args.directories:
        try:
            result = evaluate_run(directory, bag_factory=PortableBag)
            result['source_directory'] = str(directory)
        except Exception as error:
            result = dict(passed=False, source_directory=str(directory),
                          error=str(error))
        results.append(result)
        print('{}: {}'.format(directory.name, 'PASS' if result['passed'] else 'FAIL'))
    if args.output:
        # Exclusive creation prevents accidental replacement of original evidence.
        with args.output.open('x', encoding='utf-8') as output:
            json.dump(dict(audit_type='existing recorded evidence, not new flights',
                           results=results), output, indent=2, allow_nan=False)
            output.write('\n')
    return 0 if all(row['passed'] for row in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())

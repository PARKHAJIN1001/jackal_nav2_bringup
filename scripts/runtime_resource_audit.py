#!/usr/bin/env python3
"""Bounded, read-only Linux RSS/swap/UDP sampler. Never signals monitored processes."""

import argparse
import json
import math
from pathlib import Path
import time


def numeric_fields(text, names):
    result = {}
    for line in text.splitlines():
        key, sep, value = line.partition(':')
        if sep and key in names:
            result[key] = int(value.split()[0])
    return result


def process_snapshot(pid, proc=Path('/proc')):
    base = proc / str(pid)
    # comm may contain spaces or ')'; fields after the last ')' start at field 3.
    before = (base / 'stat').read_text().rsplit(')', 1)[1].split()[19]
    values = numeric_fields((base / 'status').read_text(),
                            {'VmRSS', 'VmHWM', 'VmSwap', 'RssAnon', 'RssFile', 'RssShmem'})
    after = (base / 'stat').read_text().rsplit(')', 1)[1].split()[19]
    if before != after:
        raise ValueError('PID reused during sample')
    return {'pid': pid, 'start_ticks': int(before), 'memory_kib': values}


def udp_counters(text):
    lines = [line.split()[1:] for line in text.splitlines() if line.startswith('Udp:')]
    if len(lines) != 2 or len(lines[0]) != len(lines[1]):
        raise ValueError('missing or malformed UDP counters')
    return {name: int(value) for name, value in zip(*lines)
            if name in {'InDatagrams', 'InErrors', 'RcvbufErrors', 'SndbufErrors'}}


def counter_delta(first, current):
    """Report reset/missing counters as unknown, never as negative packet loss."""
    return {key: current[key] - value if key in current and current[key] >= value else None
            for key, value in first.items()}


def sample(pids, identities, proc=Path('/proc')):
    report = {'wall_sec': time.time(), 'monotonic_sec': time.monotonic(),
              'processes': [], 'errors': []}
    for pid in pids:
        try:
            item = process_snapshot(pid, proc)
            expected = identities.setdefault(pid, item['start_ticks'])
            if item['start_ticks'] != expected:
                raise ValueError('PID reused; refusing to sample another process')
            report['processes'].append(item)
        except (OSError, ValueError, IndexError) as exc:
            report['errors'].append({'pid': pid, 'error': str(exc)})
    try:
        report['memory_kib'] = numeric_fields(
            (proc / 'meminfo').read_text(), {'MemAvailable', 'SwapTotal', 'SwapFree'})
        report['udp'] = udp_counters((proc / 'net/snmp').read_text())
    except (OSError, ValueError) as exc:
        report['errors'].append({'system': str(exc)})
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, action='append', required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--output', type=Path, required=True, help='new JSONL file')
    args = parser.parse_args(argv)
    if (any(pid <= 0 for pid in args.pid)
            or not all(math.isfinite(v) and v > 0 for v in (args.duration, args.interval))
            or args.interval < 0.1 or args.duration / args.interval > 10000):
        parser.error('positive PIDs/times, interval >= 0.1s, at most 10000 intervals required')
    identities, baseline = {}, None
    started = time.monotonic()
    code = 0
    with args.output.open('x', encoding='utf-8') as stream:
        try:
            while True:
                entry = sample(sorted(set(args.pid)), identities)
                if 'udp' in entry:
                    if baseline is None:
                        baseline = entry['udp']
                    entry['udp_delta_since_start'] = counter_delta(baseline, entry['udp'])
                stream.write(json.dumps(entry) + '\n')
                stream.flush()
                if entry['errors']:
                    code = 2
                    break
                remaining = args.duration - (time.monotonic() - started)
                if remaining <= 0:
                    break
                time.sleep(min(args.interval, remaining))
        except KeyboardInterrupt:
            code = 130
        stream.write(json.dumps({'recording_complete': code == 0, 'exit_code': code,
                                 'elapsed_sec': time.monotonic() - started}) + '\n')
    print(f'Read-only resource samples: {args.output} (exit {code})')
    return code


if __name__ == '__main__':
    raise SystemExit(main())

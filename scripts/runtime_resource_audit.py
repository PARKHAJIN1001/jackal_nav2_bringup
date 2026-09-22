#!/usr/bin/env python3
"""Bounded, read-only Linux RSS/swap/UDP sampler. Never signals monitored processes."""

import argparse
import json
import math
import os
from pathlib import Path
import threading
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


def udp_sockets(text):
    """Read queue sizes and per-socket drops from Linux procfs (UDP/UDP6)."""
    result = []
    for line in text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 13:
            continue
        tx, rx = fields[4].split(':')
        result.append({'inode': int(fields[9]), 'local': fields[1],
                       'remote': fields[2], 'tx_queue_bytes': int(tx, 16),
                       'rx_queue_bytes': int(rx, 16), 'drops': int(fields[-1])})
    return result


def socket_owners(pids, proc):
    owners = {}
    for pid in pids:
        try:
            for fd in (proc / str(pid) / 'fd').iterdir():
                try:
                    link = os.readlink(fd)
                    if link.startswith('socket:['):
                        owners.setdefault(int(link[8:-1]), []).append(pid)
                except (OSError, ValueError):
                    continue
        except OSError:
            continue
    return {inode: sorted(set(values)) for inode, values in owners.items()}


def process_metadata(pid, proc=Path('/proc')):
    """Record only transport environment keys; never copy unrelated secrets."""
    base = proc / str(pid)
    keys = {'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_PUBLICATION_MODE',
            'RMW_FASTRTPS_USE_QOS_FROM_XML'}
    env = {}
    for item in (base / 'environ').read_bytes().split(b'\0'):
        key, sep, value = item.partition(b'=')
        if sep and key.decode(errors='replace') in keys:
            env[key.decode()] = value.decode(errors='replace')
    libraries = sorted({
        line.split()[-1] for line in (base / 'maps').read_text().splitlines()
        if '/' in line and any(key in line for key in
                               ('libfastrtps', 'libfastdds', 'libfastcdr', 'librmw'))})
    return {'executable': os.readlink(base / 'exe'), 'transport_environment': env,
            'mapped_dds_libraries': libraries, 'xml_load_verified': False}


class ResourceRecorder:
    """One bounded sampler owned by an existing session/recorder, never a ROS node."""

    def __init__(self, output, pids, session_id, session_dir=None):
        self.output, self.pids, self.session_id = output, pids, session_id
        self.session_dir = session_dir
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.error = None

    def start(self):
        self.thread.start()
        return self

    def run(self):
        identities, metadata, baseline, sockets = {}, set(), None, {}
        try:
            with self.output.open('x', encoding='utf-8') as stream:
                while not self.done.is_set():
                    entry = sample(self.pids(), identities)
                    entry['session_id'] = self.session_id
                    if baseline is None:
                        baseline = entry.get('udp', {})
                    entry['udp_delta_since_start'] = counter_delta(baseline, entry.get('udp', {}))
                    for item in entry['processes']:
                        key = (item['pid'], item['start_ticks'])
                        if key not in metadata:
                            try:
                                item.update(process_metadata(item['pid']))
                                if item['mapped_dds_libraries']:
                                    metadata.add(key)
                            except OSError as error:
                                item['metadata_error'] = str(error)
                    for item in entry.get('udp_sockets', []):
                        key = (item['inode'], item['local'])
                        first = sockets.setdefault(key, item['drops'])
                        item['drops_delta_since_first_observed'] = (
                            item['drops'] - first if item['drops'] >= first else None)
                    if self.session_dir:
                        for name in ('stability.json', 'session.json'):
                            try:
                                entry[name] = json.loads((self.session_dir / name).read_text())
                            except (OSError, ValueError):
                                pass
                    stream.write(json.dumps(entry) + '\n')
                    stream.flush()
                    self.done.wait(1.0)
                stream.write(json.dumps({'session_id': self.session_id,
                                         'resource_sampler_stopped': True}) + '\n')
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.done.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            self.error = 'resource sampler did not finish'


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
    try:
        owners = socket_owners(pids, proc)
        report['udp_sockets'] = []
        for name in ('udp', 'udp6'):
            path = proc / 'net' / name
            if path.exists():
                for item in udp_sockets(path.read_text()):
                    item.update(protocol=name, owners=owners.get(item['inode'], []))
                    report['udp_sockets'].append(item)
    except (OSError, ValueError, IndexError) as exc:
        report['errors'].append({'sockets': str(exc)})
    return report


def compare_hosts(paths):
    """Join already captured host evidence; never contact either machine."""
    hosts, session_ids = [], []
    for path in paths:
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        samples = [row for row in rows if 'udp' in row]
        if not samples:
            raise ValueError(f'No UDP samples in {path}')
        ids = {row.get('session_id') for row in samples}
        if len(ids) != 1 or not next(iter(ids)) or ids == {'standalone'}:
            raise ValueError('Both captures require the same explicit session ID')
        session_ids.append(next(iter(ids)))
        hosts.append({
            'source': str(path), 'samples': len(samples),
            'first_wall_sec': samples[0]['wall_sec'], 'last_wall_sec': samples[-1]['wall_sec'],
            'udp_delta': counter_delta(samples[0]['udp'], samples[-1]['udp']),
            'socket_drops': samples[-1].get('udp_sockets', []),
            'stability_transitions': [
                {'wall_sec': row['wall_sec'], **row['stability.json']} for index, row in
                enumerate(samples) if 'stability.json' in row and
                (index == 0 or row['stability.json'].get('phase') !=
                 samples[index - 1].get('stability.json', {}).get('phase'))],
        })
    if len(set(session_ids)) != 1:
        raise ValueError('Session IDs differ; refusing to join unrelated runs')
    return {'session_id': session_ids[0], 'hosts': hosts,
            'meaning': 'Per-host counter changes only, not a packet-loss rate. '
                       'Wall-clock alignment requires independent time-sync evidence.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, action='append', default=[])
    parser.add_argument('--session-id', default='standalone')
    parser.add_argument('--compare', type=Path, nargs=2, metavar=('LAPTOP_JSONL', 'NUC_JSONL'))
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--output', type=Path, required=True, help='new JSONL file')
    args = parser.parse_args(argv)
    if args.compare:
        try:
            result = compare_hosts(args.compare)
        except (OSError, ValueError, KeyError) as error:
            parser.error(str(error))
        with args.output.open('x') as stream:
            stream.write(json.dumps(result, indent=2) + '\n')
        return 0
    if (any(pid <= 0 for pid in args.pid)
            or not all(math.isfinite(v) and v > 0 for v in (args.duration, args.interval))
            or args.interval < 0.1 or args.duration / args.interval > 10000):
        parser.error('positive PIDs/times, interval >= 0.1s, at most 10000 intervals required')
    identities, baseline, socket_baseline, metadata = {}, None, {}, set()
    started = time.monotonic()
    code = 0
    with args.output.open('x', encoding='utf-8') as stream:
        try:
            while True:
                entry = sample(sorted(set(args.pid)), identities)
                entry['session_id'] = args.session_id
                for item in entry['processes']:
                    key = (item['pid'], item['start_ticks'])
                    if key not in metadata:
                        try:
                            item.update(process_metadata(item['pid']))
                            if item['mapped_dds_libraries']:
                                metadata.add(key)
                        except OSError as error:
                            item['metadata_error'] = str(error)
                for item in entry.get('udp_sockets', []):
                    key = (item['inode'], item['local'])
                    first = socket_baseline.setdefault(key, item['drops'])
                    item['drops_delta_since_first_observed'] = (
                        item['drops'] - first if item['drops'] >= first else None)
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

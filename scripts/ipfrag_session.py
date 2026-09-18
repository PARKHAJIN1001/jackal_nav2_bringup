#!/usr/bin/env python3
"""Explicit temporary 128MiB setup/restore; changes only ipfrag_high_thresh."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import sys

from nav_session import IPFRAG, MIN_IPFRAG, stack_processes


STATE_DIR = Path('/run/jackal-nav2-ipfrag')
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')


def save(path, state):
    """Persist the original limit before changing the kernel."""
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as stream:
        json.dump(state, stream)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def change(action, state_path, kernel=IPFRAG, boot_file=BOOT_ID, processes=stack_processes):
    """Apply idempotently and refuse to restore while the robot stack is active."""
    current = int(kernel.read_text())
    boot = boot_file.read_text().strip()
    namespace = os.readlink('/proc/self/ns/net')
    state = json.loads(state_path.read_text()) if state_path.exists() else None
    if state and state['boot'] != boot:
        raise RuntimeError('Saved state belongs to another boot; inspect it before proceeding.')
    if state and state['network_namespace'] != namespace:
        raise RuntimeError('Saved state belongs to another network namespace.')
    if action == 'apply' and state:
        if current != state['target']:
            raise RuntimeError('Kernel limit changed outside this tool; original state retained.')
        return {'status': 'already_applied', **state}
    if action == 'apply' and current >= MIN_IPFRAG:
        return {'status': 'already_sufficient_unmanaged', 'current': current,
                'message': 'No change and no ownership claimed; use the original restore method.'}
    if action == 'restore' and state is None:
        raise RuntimeError('No saved original value; this tool will not guess a restore value.')
    if processes():
        raise RuntimeError('Stop Nav2/FAST/perception before changing the kernel limit.')
    if action == 'apply':
        state = {'boot': boot, 'network_namespace': namespace,
                 'original': current, 'target': MIN_IPFRAG}
        save(state_path, state)
        # Retain the snapshot on failure: an interrupted write may be ambiguous.
        kernel.write_text(str(MIN_IPFRAG) + '\n')
        if int(kernel.read_text()) != MIN_IPFRAG:
            raise RuntimeError('Kernel limit verification failed; original state retained.')
        return {'status': 'applied', **state}
    if current not in (state['target'], state['original']):
        raise RuntimeError('Kernel limit changed outside this tool; refusing to overwrite it.')
    if current != state['original']:
        kernel.write_text(str(state['original']) + '\n')
    if int(kernel.read_text()) != state['original']:
        raise RuntimeError('Restore verification failed; original state retained.')
    state_path.unlink()
    return {'status': 'restored', 'current': state['original']}


def main(argv=None):
    """Keep runtime-only settings independent from any terminal lifetime."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'apply', 'restore'])
    args = parser.parse_args(argv)
    try:
        if args.action == 'status':
            print(json.dumps({'ipfrag_high_thresh': int(IPFRAG.read_text()),
                              'network_namespace': os.readlink('/proc/self/ns/net')}))
            return 0
        if os.geteuid() != 0:
            raise RuntimeError('Run apply/restore with sudo; status does not need sudo.')
        STATE_DIR.mkdir(mode=0o700, exist_ok=True)
        stat = STATE_DIR.lstat()
        if STATE_DIR.is_symlink() or stat.st_uid != 0 or stat.st_mode & 0o077:
            raise RuntimeError(f'Unsafe state directory: {STATE_DIR}')
        with (STATE_DIR / 'lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            print(json.dumps(change(args.action, STATE_DIR / 'state.json'), indent=2))
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        print(f'ipfrag_session: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

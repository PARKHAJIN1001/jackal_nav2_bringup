"""Read-only local/SSH clock comparison; no clock adjustment."""
import argparse
import ctypes
import json
from pathlib import Path
import selectors
import shlex
import subprocess
import time

SLEW = '''import ctypes
class Timeval(ctypes.Structure):
    _fields_ = [('tv_sec', ctypes.c_long), ('tv_usec', ctypes.c_long)]
old = Timeval()
libc = ctypes.CDLL(None, use_errno=True)
rc = libc.adjtime(None, ctypes.byref(old))
slew = old.tv_sec + old.tv_usec / 1e6 if rc == 0 else None
'''

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
local = {}
exec(SLEW, local)
remote = SLEW + '''
import sys, time, json
print(json.dumps({'pending_slew_sec': slew}), flush=True)
for line in sys.stdin:
    print(time.time_ns(), flush=True)
'''
process = subprocess.Popen(
    ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'jackal',
     'python3 -u -c ' + shlex.quote(remote)],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
selector = selectors.DefaultSelector()
selector.register(process.stdout, selectors.EVENT_READ)

def read_line():
    if not selector.select(timeout=8):
        raise TimeoutError('SSH clock probe did not reply')
    line = process.stdout.readline()
    if not line:
        raise RuntimeError('SSH clock probe exited early')
    return line

try:
    nuc = json.loads(read_line())
    samples = []
    for _ in range(10):
        before, mono = time.time_ns(), time.monotonic_ns()
        process.stdin.write('sample\n')
        process.stdin.flush()
        remote_ns = int(read_line())
        after = time.time_ns()
        samples.append({'nuc_minus_laptop_sec': (remote_ns - (before + after) / 2) / 1e9,
                        'roundtrip_sec': (time.monotonic_ns() - mono) / 1e9})
        time.sleep(0.1)
    report = {'local_ros_wall_sec': time.time(), 'local_pending_slew_sec': local['slew'],
              'nuc_pending_slew_sec': nuc['pending_slew_sec'], 'samples': samples,
              'best_roundtrip_sample': min(samples, key=lambda s: s['roundtrip_sec'])}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
finally:
    selector.close()
    process.stdin.close()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3)

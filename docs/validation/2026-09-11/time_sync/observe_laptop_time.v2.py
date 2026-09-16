"""Read-only chrony and LAN NTP observation; no system clock adjustments."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time

directory = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--samples', type=int, default=9)
args = parser.parse_args()
remote_probe = '''
import json, socket, struct, time
message = bytearray(48)
message[0] = 0x23
epoch = 2208988800
t1 = time.time()
struct.pack_into('!II', message, 40, int(t1) + epoch, int((t1 % 1) * 2**32))
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(3)
    sock.connect(('192.168.50.1', 123))
    sock.send(message)
    data = sock.recv(512)
t4 = time.time()
if len(data) < 48 or data[24:32] != message[40:48] or data[0] & 7 != 4:
    raise RuntimeError('Invalid or unmatched NTP response')
def timestamp(start):
    sec, frac = struct.unpack_from('!II', data, start)
    return sec - epoch + frac / 2**32
t2, t3 = timestamp(32), timestamp(40)
print(json.dumps({'leap': data[0] >> 6, 'stratum': data[1],
 'served_ntp_minus_nuc_system_sec': ((t2-t1)+(t3-t4))/2,
 'roundtrip_sec': (t4-t1)-(t3-t2),
 'root_delay_sec': struct.unpack_from('!i', data, 4)[0]/65536,
 'root_dispersion_sec': struct.unpack_from('!I', data, 8)[0]/65536}))
'''
observations = []
started = time.monotonic()
for index in range(args.samples):
    row = {'elapsed_sec': time.monotonic() - started, 'wall_time': time.time()}
    for label, command in (
        ('tracking', ['chronyc', '-n', '-c', 'tracking']),
        ('sources', ['chronyc', '-n', '-c', 'sources']),
        ('nuc_timesync', ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'jackal',
                          'timedatectl show-timesync --all']),
        ('nuc_ntp', ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'jackal',
                     'python3 -c ' + shlex.quote(remote_probe)]),
        ('host_clock', [sys.executable, str(directory / 'clock_probe.py'), '--output',
                        str(args.output.with_suffix('.latest_host_clock.json'))]),
    ):
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=15)
            row[label] = {'returncode': result.returncode, 'stdout': result.stdout.strip(),
                          'stderr': result.stderr.strip()}
        except subprocess.TimeoutExpired:
            row[label] = {'error': 'timeout'}
    observations.append(row)
    args.output.write_text(json.dumps(observations, indent=2)+'\n')
    print(json.dumps(row), flush=True)
    if index != args.samples - 1:
        time.sleep(16)

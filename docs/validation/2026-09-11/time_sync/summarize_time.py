"""Summarize read-only observations without changing any time/service settings."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re

parser = argparse.ArgumentParser()
parser.add_argument('input', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rows = json.loads(args.input.read_text())
offsets, residuals, distances, skews, packets, errors = [], [], [], [], [], []
for index, row in enumerate(rows):
    try:
        for name in ('tracking', 'nuc_ntp'):
            if row[name].get('returncode') != 0:
                raise ValueError(f'{name} failed')
        tracking = row['tracking']['stdout'].split(',')
        ntp = json.loads(row['nuc_ntp']['stdout'])
        if tracking[13] != 'Normal' or ntp['leap'] != 0:
            raise ValueError('not synchronised')
        offsets.append(-ntp['laptop_minus_nuc_sec'])
        residuals.append(abs(float(tracking[4])))
        distances.append(ntp['root_dispersion_sec'] + ntp['root_delay_sec'] / 2)
        skews.append(float(tracking[9]))
        state = row.get('nuc_timesync', {})
        if state:
            if state.get('returncode') != 0:
                raise ValueError('NUC timesync query failed')
            count = re.search(r'PacketCount=(\d+)', state['stdout'])
            if count is None:
                raise ValueError('NUC packet count missing')
            packets.append(int(count.group(1)))
    except (KeyError, ValueError, TypeError, IndexError) as exc:
        errors.append({'index': index, 'error': str(exc)})

summary = {
    'source': str(args.input), 'samples': len(rows),
    'start_utc': datetime.fromtimestamp(rows[0]['wall_time'], timezone.utc).isoformat(),
    'duration_sec': rows[-1]['elapsed_sec'] - rows[0]['elapsed_sec'],
    'nuc_minus_laptop_sec_range': [min(offsets), max(offsets)],
    'max_abs_offset_sec': max(map(abs, offsets)),
    'max_chrony_remaining_correction_sec': max(residuals),
    'max_lan_response_root_distance_sec': max(distances),
    'tracking_skew_ppm_range': [min(skews), max(skews)],
    'packet_counts': packets,
    'offset_over_20ms_samples': sum(abs(value) > 0.020 for value in offsets),
    'remaining_over_5ms_samples': sum(value > 0.005 for value in residuals),
    'root_distance_over_5s_samples': sum(value > 5 for value in distances),
    'errors': errors,
    'meaning': 'Sampled pre-ROS window only; not continuous/reboot/ROS validation',
}
args.output.write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))

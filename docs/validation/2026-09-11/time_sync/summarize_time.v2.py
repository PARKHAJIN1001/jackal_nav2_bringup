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
host_offsets, host_pending, host_roundtrips = [], [], []
ignored_packets = set()
for index, row in enumerate(rows):
    try:
        for name in ('tracking', 'nuc_ntp'):
            if row[name].get('returncode') != 0:
                raise ValueError(f'{name} failed')
        tracking = row['tracking']['stdout'].split(',')
        ntp = json.loads(row['nuc_ntp']['stdout'])
        if tracking[13] != 'Normal' or ntp['leap'] != 0:
            raise ValueError('not synchronised')
        # Legacy files misnamed chrony's served estimate as the laptop OS clock.
        served = ntp.get('served_ntp_minus_nuc_system_sec', ntp.get('laptop_minus_nuc_sec'))
        offsets.append(-served)
        residuals.append(abs(float(tracking[4])))
        distances.append(ntp['root_dispersion_sec'] + ntp['root_delay_sec'] / 2)
        skews.append(float(tracking[9]))
        if 'host_clock' in row:
            if row['host_clock'].get('returncode') != 0:
                raise ValueError('direct host clock comparison failed')
            clock = json.loads(row['host_clock']['stdout'])
            host_offsets.append(clock['best_roundtrip_sample']['nuc_minus_laptop_sec'])
            host_roundtrips.append(clock['best_roundtrip_sample']['roundtrip_sec'])
            for key in ('local_pending_slew_sec', 'nuc_pending_slew_sec'):
                if clock[key] is None:
                    raise ValueError('pending slew unavailable')
                host_pending.append(abs(clock[key]))
        state = row.get('nuc_timesync', {})
        if state:
            if state.get('returncode') != 0:
                raise ValueError('NUC timesync query failed')
            count = re.search(r'PacketCount=(\d+)', state['stdout'])
            if count is None:
                raise ValueError('NUC packet count missing')
            packets.append(int(count.group(1)))
            if 'Ignored=yes' in state['stdout']:
                ignored_packets.add(int(count.group(1)))
    except (KeyError, ValueError, TypeError, IndexError) as exc:
        errors.append({'index': index, 'error': str(exc)})

summary = {
    'source': str(args.input), 'samples': len(rows),
    'start_utc': datetime.fromtimestamp(rows[0]['wall_time'], timezone.utc).isoformat(),
    'duration_sec': rows[-1]['elapsed_sec'] - rows[0]['elapsed_sec'],
    'nuc_system_minus_served_ntp_sec_range': [min(offsets), max(offsets)],
    'max_abs_served_ntp_offset_sec': max(map(abs, offsets)),
    'direct_host_samples': len(host_offsets),
    'nuc_minus_laptop_system_sec_range': [min(host_offsets), max(host_offsets)]
        if host_offsets else None,
    'max_abs_host_offset_sec': max(map(abs, host_offsets)) if host_offsets else None,
    'max_host_roundtrip_sec': max(host_roundtrips) if host_roundtrips else None,
    'max_adjtime_pending_sec': max(host_pending) if host_pending else None,
    'max_chrony_remaining_correction_sec': max(residuals),
    'max_lan_response_root_distance_sec': max(distances),
    'tracking_skew_ppm_range': [min(skews), max(skews)],
    'packet_counts': packets,
    'ignored_packet_counts_observed': sorted(ignored_packets),
    'host_offset_over_20ms_samples': sum(abs(value) > 0.020 for value in host_offsets)
        if host_offsets else None,
    'remaining_over_5ms_samples': sum(value > 0.005 for value in residuals),
    'root_distance_over_5s_samples': sum(value > 5 for value in distances),
    'errors': errors,
    'meaning': 'Served NTP estimate is NOT laptop OS time. Only host_clock measures '
        'OS-to-OS offset. Sampled pre-ROS window, not continuous/reboot/ROS validation.',
}
args.output.write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))

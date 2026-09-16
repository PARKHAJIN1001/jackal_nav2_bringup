# Time validation artifacts — 2026-09-11

Raw observations are retained, including early failed windows. No artifact here
constitutes motion approval, reboot validation or long-term clock acceptance.

## Important measurement correction (17:10 KST)

In legacy `laptop_time_*.json` files, `nuc_ntp.stdout` JSON uses the misleading
key `laptop_minus_nuc_sec`. It actually measures **chrony's served NTP estimate
minus the NUC OS clock**, not the laptop OS clock minus the NUC OS clock.
Chrony can serve corrected time while its own OS clock is still slewing.
Legacy raw files are unchanged. Regenerated summaries explicitly distinguish
`served_ntp` from direct `host` measurements; missing direct data is `null`,
never inferred to pass from the NTP reply alone.

`clock_*.json` files use direct SSH round-trip OS-clock comparisons. Each contains
10 samples, the minimum-round-trip sample and both `adjtime` pending values.
These are not absolute UTC measurements; RTT/asymmetry and scheduling remain
measurement limitations. Chrony's remaining correction is recorded separately.

`laptop_time_os_verified.json` records the direct `host_clock` measurement in
every row alongside `tracking`, served NTP data and NUC timesync state.
Its NTP field is named `served_ntp_minus_nuc_system_sec`.

The `.v2.py` observer/summary scripts implement this distinction. They are
read-only diagnostics requiring the existing `jackal` SSH alias and local
chronyc; they never set clocks or start/stop ROS services. Both observer
versions use an explicit output path; use a fresh path to preserve prior runs.

Reference: [chrony 4.2 System time](https://chrony-project.org/doc/4.2/chronyc.html#tracking).

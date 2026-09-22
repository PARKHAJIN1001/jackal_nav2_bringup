# Disconnected verification — startup stability

Recorded: 2026-09-21T05:08:00.252618+00:00

- Nav2 offline Python tests: **261 passed** (`nav_unit.txt`).
- Network offline Python tests: **44 passed** (`network_unit.txt`).
- Python lint: **passed**, using installed Humble ament_flake8 configuration; empty `lint.txt` means no findings.
- AST parse: **52 files passed**, including opt-in integration sources.
- Source fingerprints: `source_sha256.json`.

The packages were tested in separate pytest invocations because their test_config_and_launch.py
module names collide in the installed launch_testing pytest collector. Both complete suites passed.

Commands (with existing ROS/workspace environment sourced):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test/test_*.py
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider ../jackal_network_bringup/test/test_*.py
```

Coverage includes continuous 180s hold and reset, distinct 600s deadlines, stale heartbeat,
initialpose/lifecycle ordering, critical-process shutdown and failure evidence, profile forwarding,
owned-process cleanup using local dummy Python children, XML/kernel policy mismatch, process
transport environment evidence, per-socket drops and two-host session joins.

No ROS node/launch integration, Jackal/NUC connection, driver, robot command, kernel/time-sync/XML
change, package build/install or hardware restart was performed. ROS imports and launch-description
construction are offline checks, not runtime acceptance. Test-only synthetic launches were adapted
but not executed. Hardware readiness, braking, restart success rate and DDS traffic remain unverified.
See [operator procedure](../../../Startup_Stability.md).

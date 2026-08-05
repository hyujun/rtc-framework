#!/usr/bin/env python3
"""Cross-language equivalence for the generated thread layout (issue #153 M1).

``gen_thread_layout.py --check`` proves the checked-in artifacts are what the
manifest currently generates. That is a *text* comparison. This test asserts the
stronger property the acceptance criteria ask for: that the manifest, the shell
helpers and the Python launch mirror produce the **same normalised
role -> slot/policy/priority/nice result** for every physical core count in
1..17 and 24 -- i.e. that the emitters are semantically equivalent, not just
that the files are fresh.

The three axes are exercised differently on purpose:

* manifest -- parsed directly through the generator's loader
* shell    -- the helper functions are actually *executed* under bash, so a
              broken case arm or a quoting bug shows up here and not only as a
              text diff
* Python   -- the module is imported and called

The C++ axis is covered by ``rtc_base/test/test_thread_layout_tiers.cpp``, which
sweeps the same core counts against hand-written literals; it needs a compiler,
so it lives with the code it compiles rather than here.

Boundaries covered: 5/6, 7/8, 9/10, 11/12, 13/14, 15/16 (both sides of each),
plus the clamp above 16 and the degraded tier below 6.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "repo_scripts" / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "rtc_tools"))

RT_COMMON = REPO_ROOT / "repo_scripts" / "scripts" / "lib" / "rt_common.sh"

# Sampled core counts: every value across the tier boundaries plus the clamp.
CORE_COUNTS = list(range(1, 18)) + [24]

ROLES = (
    "rt_control",
    "rt_callback",
    "mpc_main",
    "mpc_worker_0",
    "mpc_worker_1",
    "arm_driver",
    "hand_driver",
    "nrt_callback",
    "nrt_logging",
    "sim_thread",
    "viewer",
)


class Failures:
    """Collects mismatches so one run reports every axis, not just the first."""

    def __init__(self) -> None:
        self.items: list[str] = []

    def check(self, label: str, expected: object, actual: object) -> None:
        if expected != actual:
            self.items.append(f"{label}: expected {expected!r}, got {actual!r}")

    def report(self) -> int:
        if not self.items:
            print(f"thread layout equivalence: OK ({len(CORE_COUNTS)} core counts)")
            return 0
        print(f"thread layout equivalence FAILED ({len(self.items)} mismatches):", file=sys.stderr)
        for item in self.items:
            print(f"  - {item}", file=sys.stderr)
        return 1


def shell_query(script: str) -> str:
    """Run a snippet with rt_common.sh sourced and return its stdout."""
    result = subprocess.run(
        ["bash", "-c", f'source "{RT_COMMON}" >/dev/null 2>&1\n{script}'],
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    if result.returncode != 0:
        raise RuntimeError(f"shell snippet failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout.strip()


def shell_batch(core_counts: list[int]) -> dict[int, dict[str, str]]:
    """One bash invocation for the whole sweep -- per-call spawning is far slower.

    Returns ``{ncpu: {query: value}}``. A role that does not exist on a tier is
    reported as the literal ``ABSENT`` so the caller can distinguish "missing"
    from "empty string", which is what a silently failing helper would return.
    """
    lines = []
    for ncpu in core_counts:
        for role in ROLES:
            lines.append(
                f"if spec=$(get_role_spec {role} {ncpu} 2>/dev/null); then "
                f'echo "{ncpu}|role:{role}|$spec"; else echo "{ncpu}|role:{role}|ABSENT"; fi'
            )
        lines.append(f'echo "{ncpu}|mpc_cores|$(get_mpc_cores {ncpu})"')
        lines.append(f'echo "{ncpu}|rt_cores|$(get_rt_cores {ncpu})"')
        lines.append(f'echo "{ncpu}|nrt_cores|$(get_nrt_cores {ncpu})"')
        lines.append(f'echo "{ncpu}|arm_slot|$(get_arm_driver_slot {ncpu})"')
        lines.append(f'echo "{ncpu}|hand_slot|$(get_hand_driver_slot {ncpu})"')
        lines.append(f'echo "{ncpu}|os_cores|$(get_os_cores)"')
        lines.append(
            f'echo "{ncpu}|expected|$(rtc_expected_threads {ncpu} | tr "\\n" " " | sed "s/ $//")"'
        )

    out = shell_query("\n".join(lines))
    table: dict[int, dict[str, str]] = {n: {} for n in core_counts}
    for line in out.splitlines():
        ncpu_s, key, value = line.split("|", 2)
        table[int(ncpu_s)][key] = value
    return table


def main() -> int:
    from gen_thread_layout import load_manifest, mpc_cores, nrt_cores, rt_cores  # noqa: PLC0415

    from rtc_tools.launch.thread_layout_generated import (  # noqa: PLC0415
        ROLE_SPECS,
        tier_for_cores,
    )

    manifest = load_manifest()
    shell = shell_batch(CORE_COUNTS)
    fail = Failures()

    for ncpu in CORE_COUNTS:
        tier = manifest.tier_for(ncpu)
        py_specs = ROLE_SPECS[tier_for_cores(ncpu)]

        # The three sides must agree on which tier this core count maps to.
        fail.check(f"ncpu={ncpu} tier(python vs manifest)", tier.id, tier_for_cores(ncpu))

        for role in ROLES:
            spec = tier.specs.get(role)
            shell_value = shell[ncpu][f"role:{role}"]

            if spec is None:
                # Absent in the manifest => absent in both mirrors. An mpc_worker
                # that leaks onto a tier without it makes the verifier hunt a
                # thread that is never created.
                fail.check(f"ncpu={ncpu} {role} shell absence", "ABSENT", shell_value)
                if role in py_specs:
                    fail.items.append(f"ncpu={ncpu} {role}: present in Python, absent in manifest")
                continue

            expected = f"{spec.slot} {spec.policy_cpp} {spec.priority} {spec.nice}"
            fail.check(f"ncpu={ncpu} {role} shell spec", expected, shell_value)

            py = py_specs.get(role)
            if py is None:
                fail.items.append(f"ncpu={ncpu} {role}: absent in Python, present in manifest")
                continue
            fail.check(f"ncpu={ncpu} {role} python slot", spec.slot, py.slot)
            fail.check(f"ncpu={ncpu} {role} python policy", spec.policy, py.policy)
            fail.check(f"ncpu={ncpu} {role} python priority", spec.priority, py.priority)
            fail.check(f"ncpu={ncpu} {role} python nice", spec.nice, py.nice)

        # Derived CSV helpers -- the shape the shell consumers actually read.
        fail.check(
            f"ncpu={ncpu} get_mpc_cores",
            ",".join(str(v) for v in mpc_cores(manifest, tier)),
            shell[ncpu]["mpc_cores"],
        )
        fail.check(
            f"ncpu={ncpu} get_rt_cores",
            ",".join(str(v) for v in rt_cores(manifest, tier)),
            shell[ncpu]["rt_cores"],
        )
        fail.check(
            f"ncpu={ncpu} get_nrt_cores",
            ",".join(str(v) for v in nrt_cores(manifest, tier)),
            shell[ncpu]["nrt_cores"],
        )
        fail.check(
            f"ncpu={ncpu} arm slot", str(tier.specs["arm_driver"].slot), shell[ncpu]["arm_slot"]
        )
        fail.check(
            f"ncpu={ncpu} hand slot", str(tier.specs["hand_driver"].slot), shell[ncpu]["hand_slot"]
        )
        fail.check(f"ncpu={ncpu} os slot", str(manifest.os_slot), shell[ncpu]["os_cores"])

        # The launch mirror's public API is what integrated_bringup imports.
        from rtc_tools.launch.thread_layout import select_thread_layout  # noqa: PLC0415

        layout = select_thread_layout(ncpu)
        fail.check(
            f"ncpu={ncpu} launch arm", tier.specs["arm_driver"].slot, layout.arm_driver_core
        )
        fail.check(
            f"ncpu={ncpu} launch hand", tier.specs["hand_driver"].slot, layout.hand_driver_core
        )
        fail.check(
            f"ncpu={ncpu} launch sim", tier.specs["sim_thread"].slot, layout.sim_thread_core
        )
        fail.check(f"ncpu={ncpu} launch viewer", tier.specs["viewer"].slot, layout.viewer_core)
        fail.check(
            f"ncpu={ncpu} launch rt_callback",
            tier.specs["rt_callback"].slot,
            layout.rt_callback_core,
        )

        # Verifier table: only in-process controller threads, in manifest order.
        rows = []
        for key in manifest.verifier_order:
            spec = tier.specs.get(key)
            if spec is None:
                continue
            name = key
            row = f"{name}:{spec.slot}:{spec.policy_verifier}:{spec.priority}"
            if key.startswith("mpc_worker_"):
                row += ":optional"
            rows.append(row)
        fail.check(f"ncpu={ncpu} rtc_expected_threads", " ".join(rows), shell[ncpu]["expected"])

    return fail.report()


if __name__ == "__main__":
    sys.exit(main())

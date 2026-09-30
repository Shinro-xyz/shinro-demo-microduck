"""Compare the policy's three ways to run: onnxruntime, the shinro adapter, and the deployment host.

Every backend is probed in its **own process** (``scripts/backend_probe.py``) so
the measurements cannot contaminate each other, then this script adds what only
a parent can:

* **on-disk footprint** — what each route requires to be *installed*, including
  the transitive wheels (onnxruntime pulls numpy + OpenBLAS; the deployment host
  pulls nothing);
* **numerical agreement** — the compiled kernel vs onnxruntime on the same
  observations, in one process;
* **on-target comparison** — the Pi 3 B+ numbers for the same kernel, read from
  ``docs/target_measurements.json`` (measured by ``shinro-bench``; not
  reproducible from here).

    python scripts/compare_backends.py                    # table to stdout
    python scripts/compare_backends.py --samples 5000 --out build/backend_comparison.md

Read the latency column carefully: this is a *Python* host on an x86 laptop,
where ~30 µs of interpreter/ctypes overhead sits in front of a ~197 k-parameter
MLP, so all three land within a few percent. The deployment host's case is
footprint, dependency-free embedding, a zero-allocation tick loop, and
bit-reproducible f64 arithmetic — see README.md and the on-target numbers.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
POLICY_ONNX = REPO / "models" / "microduck" / "BEST_alpha_walking.onnx"
ARTIFACT = REPO / "build" / "compiled_policy"
KERNEL = ARTIFACT / "lib" / "lib_neural_network.so"
TARGET_RESULTS = REPO / "docs" / "target_measurements.json"

BACKENDS = ("onnxruntime", "shinro-adapter", "microduck-policy")

#: Backend -> the installed distribution closure it drags in (site-packages dirs).
DISK_CLOSURES = {
    "onnxruntime": ("onnxruntime", "numpy", "numpy.libs"),
    "shinro-adapter": ("numpy", "numpy.libs"),
    "microduck-policy": (),
}

#: One-line description of what each backend is, for the report.
BACKEND_ROLE = {
    "onnxruntime": "reference runtime on the .onnx",
    "shinro-adapter": "framework's generic compiled backend (parity reference)",
    "microduck-policy": "**deployment host** — ctypes + stdlib, preallocated buffers",
}


def dir_size_mb(path: Path) -> float:
    """Total size of a directory tree, in MB."""
    if not path.exists():
        return 0.0
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file()) / 1e6


def site_packages() -> Path:
    """The site-packages directory this interpreter resolves."""
    import sysconfig

    return Path(sysconfig.get_paths()["purelib"])


def disk_footprint() -> dict[str, dict]:
    """Installed size per backend: the policy artifact plus its dependency closure."""
    sp = site_packages()
    out: dict[str, dict] = {}
    for backend, dists in DISK_CLOSURES.items():
        dep_mb = sum(dir_size_mb(sp / d) for d in dists)
        artifact_mb = (POLICY_ONNX.stat().st_size if backend == "onnxruntime" else KERNEL.stat().st_size) / 1e6
        out[backend] = {
            "artifact_mb": artifact_mb,
            "dependency_mb": dep_mb,
            "total_mb": artifact_mb + dep_mb,
            "distributions": ", ".join(dists) or "— (stdlib only)",
        }
    return out


def run_probe(backend: str, samples: int, warmup: int, cpu: int | None, repeats: int) -> dict:
    """Spawn the isolated probe process N times and keep the best stable snapshot.

    Pin to one CPU: this class of host (Intel P-core/E-core hybrids) migrates a
    process between cores whose clock differs by ~2x, which swings per-call
    latency by that much. Pinning removes it — without pinning, the same backend
    measured 37 µs and 67 µs on consecutive runs.

    Repeats guard against a noisy neighbour: latency is the **minimum** over
    repeats (the latency floor, the standard way to report this), while RSS and
    the mapped-object list come from the run with the lowest median.
    """
    runs: list[dict] = []
    for _ in range(max(1, repeats)):
        cmd = [sys.executable, str(HERE / "backend_probe.py"), "--backend", backend, "--samples", str(samples), "--warmup", str(warmup)]
        if cpu is not None:
            cmd = ["taskset", "-c", str(cpu), *cmd]
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if proc.returncode != 0:
            raise RuntimeError(f"probe {backend} failed ({proc.returncode}):\n{proc.stderr[-2000:]}")
        lines = [ln for ln in proc.stdout.splitlines() if ln.strip().startswith("{")]
        if not lines:
            raise RuntimeError(f"probe {backend} produced no JSON:\n{proc.stdout[-2000:]}")
        runs.append(json.loads(lines[-1]))

    best = min(runs, key=lambda r: r["latency_us_median"])
    merged = dict(best)
    merged["latency_us_median"] = min(r["latency_us_median"] for r in runs)
    merged["latency_us_p99"] = min(r["latency_us_p99"] for r in runs)
    merged["latency_us_min"] = min(r["latency_us_min"] for r in runs)
    merged["time_to_ready_s"] = min(r["time_to_ready_s"] for r in runs)
    merged["throughput_hz"] = max(r["throughput_hz"] for r in runs)
    merged["repeats"] = len(runs)
    return merged


def agreement(samples: int, seed: int = 0) -> float:
    """Max |Δ| between the compiled kernel and onnxruntime on shared observations.

    Both run in this one process, which is why this lives here and not in the
    probes: it is a correctness check, not a footprint measurement. The kernel is
    driven through :class:`~shinro_demo_microduck.policy.MicroduckPolicy`, i.e.
    the shipped host; the gap is f32 (ONNX runtime) vs f64 (kernel) arithmetic.
    """
    import numpy as np
    import onnxruntime as ort

    sys.path.insert(0, str(REPO / "src"))
    from shinro_demo_microduck.policy import MicroduckPolicy

    policy = MicroduckPolicy(ARTIFACT)
    session = ort.InferenceSession(str(POLICY_ONNX), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(seed)
    worst = 0.0
    for _ in range(samples):
        obs = rng.normal(0.0, 0.3, policy.n_obs)
        obs[3:6] = [0.0, 0.0, -1.0]  # keep the pose plausible
        action = np.frombuffer(policy.step(obs), dtype=np.float64)
        reference = session.run(None, {"obs": obs.reshape(1, -1).astype(np.float32)})[0].ravel()
        worst = max(worst, float(np.max(np.abs(action - reference))))
    return worst


def load_target() -> dict | None:
    """The on-target (Pi) results, if the data file is present."""
    if not TARGET_RESULTS.exists():
        return None
    return json.loads(TARGET_RESULTS.read_text())


def _table(headers: list[str], rows: list[list[str]]) -> str:
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    line = lambda cells: "| " + " | ".join(c.ljust(w) for c, w in zip(cells, widths)) + " |"  # noqa: E731
    return "\n".join([line(headers), "|" + "|".join("-" * (w + 2) for w in widths) + "|", *(line(r) for r in rows)])


def report(
    probes: dict[str, dict],
    disk: dict[str, dict],
    worst: float,
    samples: int,
    agreement_samples: int,
    cpu: int | None,
    target: dict | None,
) -> str:
    """Render the markdown report."""
    pin = f"pinned to CPU {cpu}" if cpu is not None else "NOT pinned"
    parts = [
        "# Policy backend comparison",
        "",
        f"_{samples} timed calls × {probes[BACKENDS[0]]['repeats']} repeats per backend, each backend in its own process, {pin}._",
        "",
        "| backend | role |",
        "| ------- | ---- |",
        *(f"| `{b}` | {BACKEND_ROLE[b]} |" for b in BACKENDS),
        "",
    ]

    parts.append("## On-disk footprint (required to run)")
    parts.append("")
    parts.append(
        _table(
            ["backend", "policy artifact", "dependencies", "installed total", "distributions"],
            [
                [
                    b,
                    f"{disk[b]['artifact_mb']:.2f} MB",
                    f"{disk[b]['dependency_mb']:.1f} MB",
                    f"**{disk[b]['total_mb']:.1f} MB**",
                    disk[b]["distributions"],
                ]
                for b in BACKENDS
            ],
        )
    )

    parts += ["", "## Process footprint", ""]
    parts.append(
        _table(
            ["backend", "baseline RSS", "RSS after load", "Δ over baseline", "mapped total", "mapped runtime libs"],
            [
                [
                    b,
                    f"{probes[b]['rss_baseline_kb'] / 1024:.1f} MB",
                    f"{probes[b]['rss_loaded_kb'] / 1024:.1f} MB",
                    f"**+{(probes[b]['rss_loaded_kb'] - probes[b]['rss_baseline_kb']) / 1024:.1f} MB**",
                    f"{probes[b]['mapped_total_mb']:.1f} MB",
                    f"{probes[b]['mapped_runtime_mb']:.1f} MB",
                ]
                for b in BACKENDS
            ],
        )
    )
    parts.append("")
    parts.append("`mapped runtime libs` = bytes of Python/onnxruntime/numpy/scipy shared objects in the address space.")
    parts.append("For `microduck-policy` that is the interpreter itself — the kernel adds one 0.8 MB `.so` and nothing else.")

    parts += ["", "## Time to first inference and per-call latency", ""]
    parts.append(
        _table(
            ["backend", "time to ready", "median / call", "p99 / call", "min / call", "throughput"],
            [
                [
                    b,
                    f"**{probes[b]['time_to_ready_s'] * 1000:.1f} ms**",
                    f"{probes[b]['latency_us_median']:.1f} µs",
                    f"{probes[b]['latency_us_p99']:.1f} µs",
                    f"{probes[b]['latency_us_min']:.1f} µs",
                    f"{probes[b]['throughput_hz']:,.0f} Hz",
                ]
                for b in BACKENDS
            ],
        )
    )

    parts += ["", "## Numerical agreement (kernel vs onnxruntime)", ""]
    parts.append(f"max |Δaction| over {agreement_samples} shared observations: **{worst:.2e}** (f32 runtime vs f64 kernel).")

    if target:
        p = target["provenance"]
        t, m, d = target["timing"], target["memory"], target["disk"]
        parts += ["", "## On target hardware — not measured here", ""]
        parts.append(
            f"The same kernel cross-compiled to `aarch64` and run on **{p['host']}** by the `{p['measured_by']}` harness "
            f"({p['n_ticks']:,} ticks, {p['deadline_us']} µs deadline), from `{p['source']}`:"
        )
        parts.append("")
        parts.append(
            _table(
                ["metric", "value"],
                [
                    ["median / call", f"**{t['median_us']:.1f} µs**"],
                    ["mean / p99", f"{t['mean_us']:.1f} µs / {t['p99_us']:.1f} µs"],
                    ["min / max", f"{t['min_us']:.1f} µs / {t['max_us']:.1f} µs"],
                    ["jitter (stddev)", f"{t['jitter_stddev_us']:.1f} µs"],
                    ["deadline misses", f"{t['deadline_miss_frac'] * 100:.3f} %"],
                    ["RSS", f"{m['rss_kb'] / 1024:.1f} MB (peak {m['peak_kb'] / 1024:.1f} MB)"],
                    ["RSS growth over the run", f"**{m['rss_growth_kb']} KB over {p['n_ticks']:,} ticks**"],
                    ["page faults", f"{m['minor_faults']} minor, {m['major_faults']} major"],
                    ["disk", f"{d['so_bytes'] / 1024:.0f} KB `.so` + {d['total_bytes'] - d['so_bytes']} B manifests"],
                ],
            )
        )
        parts.append("")
        parts.extend(f"- {note}" for note in target.get("notes", []))

    parts += ["", "## Files present in the address space", ""]
    for backend in BACKENDS:
        parts.append(f"**{backend}**")
        parts.append("")
        parts.extend(f"- `{path}`" for path in probes[backend]["mapped_top"])
        parts.append("")
    return "\n".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", type=int, default=5000, help="timed calls per backend (default 5000)")
    parser.add_argument("--warmup", type=int, default=500)
    parser.add_argument("--repeats", type=int, default=3, help="probe repeats per backend; latency is the min over them")
    parser.add_argument("--cpu", type=int, default=2, help="pin every probe to this CPU with taskset (default 2; -1 to disable)")
    parser.add_argument("--out", default=None, help="write the markdown report here")
    parser.add_argument("--json", dest="json_out", default=None, help="also write raw numbers here")
    parser.add_argument("--skip-agreement", action="store_true")
    parser.add_argument("--agreement-samples", type=int, default=2000, help="observations used for the kernel-vs-onnxruntime check")
    args = parser.parse_args(argv)

    cpu = None if args.cpu is not None and args.cpu < 0 else args.cpu

    missing = [p for p in (POLICY_ONNX, KERNEL) if not p.exists()]
    if missing:
        print(f"ERROR: missing {missing[0]} — run `make compile` first", file=sys.stderr)
        return 2

    probes = {}
    for backend in BACKENDS:
        print(f"probing {backend} …", file=sys.stderr)
        probes[backend] = run_probe(backend, args.samples, args.warmup, cpu, args.repeats)

    disk = disk_footprint()
    worst = float("nan") if args.skip_agreement else agreement(args.agreement_samples)
    text = report(probes, disk, worst, args.samples, args.agreement_samples, cpu, load_target())
    print(text)

    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n")
        print(f"\nwrote {out}", file=sys.stderr)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"probes": probes, "disk": disk, "agreement_max_abs": worst}, indent=2) + "\n")
        print(f"wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

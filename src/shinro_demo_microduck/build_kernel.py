"""Build and verify the compiled Microduck kernel from Python.

    python -m shinro_demo_microduck.build_kernel              # default scenario
    python -m shinro_demo_microduck.build_kernel <scenario> --out DIR --target TRIPLE

A thin **consumer** of the editable ``shinro`` install: it reads the scenario
TOML, then calls the same package functions the ``shinro`` CLI dispatches to
(``shinro.codegen.cli.compile_scenario`` and ``shinro.codegen.verify.verify``) —
no subprocess, no re-implementation. Equivalent to ``make compile`` + ``make verify``.
"""

from __future__ import annotations

import argparse
import sys
import tomllib

from shinro.codegen.cli import compile_scenario  # the function behind `shinro build`
from shinro.codegen.verify import verify  # the function behind `shinro verify`

from shinro_demo_microduck.paths import DEFAULT_ARTIFACT, resolve_repo_path

#: The scenario ``make compile`` compiles.
DEFAULT_SCENARIO = "scenarios/microduck_walking.toml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", nargs="?", default=DEFAULT_SCENARIO, help=f"scenario TOML (default: {DEFAULT_SCENARIO})")
    parser.add_argument("--out", default=str(DEFAULT_ARTIFACT), help="output dir (default: build/compiled_policy)")
    parser.add_argument("--optimize", choices=["debug", "release"], help="override [compile].optimize")
    parser.add_argument("--target", help="override [compile].target (zig triple, e.g. aarch64-linux-gnu)")
    parser.add_argument("--native-record", help="cross build: oracle-verified native record to reference")
    args = parser.parse_args(argv)

    scenario_path = resolve_repo_path(args.scenario)
    try:
        with open(scenario_path, "rb") as handle:
            spec = tomllib.load(handle)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        print(f"cannot read scenario {scenario_path}: {exc}", file=sys.stderr)
        return 2

    # The artifact stem comes from the scenario, so the record can be located after the build.
    name = spec["compile"].get("artifact_name", "libbase")
    out_dir = resolve_repo_path(args.out)

    rc = compile_scenario(
        str(scenario_path),
        str(out_dir),
        optimize=args.optimize,
        target=args.target,
        native_record=str(resolve_repo_path(args.native_record)) if args.native_record else None,
    )
    if rc != 0:
        return rc

    record = out_dir / "lib" / f"{name}.deployment.json"
    if verify(record, graph_path=out_dir / "graph_data.zig") != 0:
        return 1

    print(f"built + verified: {out_dir / 'lib' / f'{name}.so'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

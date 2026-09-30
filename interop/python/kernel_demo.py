"""Python interop: raw ``ctypes`` load of the compiled Microduck kernel.

Same port layout and same synthetic input as the C example (see
``interop/c/kernel_demo.c``). This is the *raw* C ABI, not the shipping host —
so the four languages are calling the identical entry point. On the side (to
stderr, so stdout stays the comparison vector) it also loads the same artifact
through :class:`~shinro_demo_microduck.policy.MicroduckPolicy` and asserts the
shipping host returns the same 14 values.

Run:  python3 kernel_demo.py /path/to/lib_neural_network.so
"""

from __future__ import annotations

import ctypes
import struct
import sys
from pathlib import Path

N_IN, N_OUT, N_STATE = 61, 14, 1


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "../build/compiled_policy/lib/lib_neural_network.so")
    lib = ctypes.CDLL(str(path))
    ptr = ctypes.POINTER(ctypes.c_double)
    lib.shinro_step.argtypes = [ptr, ptr, ptr]
    lib.shinro_step.restype = None

    inp = (ctypes.c_double * N_IN)()
    out = (ctypes.c_double * N_OUT)()
    state = (ctypes.c_double * N_STATE)()
    for i in range(N_IN):
        inp[i] = ((i % 11) - 5) * 0.03125

    lib.shinro_step(inp, out, state)
    print(" ".join(f"{out[i]:.17g}" for i in range(N_OUT)))

    # Cross-check the shipping host wraps this exact ABI (best-effort: skipped
    # when the demo package is not importable in this interpreter).
    try:
        from shinro_demo_microduck.policy import MicroduckPolicy

        policy = MicroduckPolicy(path.parent.parent)
        host = struct.unpack(f"<{N_OUT}d", bytes(policy.step(list(inp))))  # little-endian f64, as the ABI
        assert list(host) == [float(out[i]) for i in range(N_OUT)], "MicroduckPolicy disagrees with the raw ABI"
        print(f"  MicroduckPolicy (the shipping host) agrees: {policy.info}", file=sys.stderr)
    except ImportError:
        print("  MicroduckPolicy cross-check skipped (package not importable here)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

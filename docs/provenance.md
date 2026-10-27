*Part of [shinro-demo-microduck](../README.md).*

# Provenance

`BEST_alpha_walking.onnx` is the ONNX export of the deployed Microduck walking
policy: Weights & Biases run `441tzs6d` @ `model_3750`, published at
[`xiaofengzi/microduck-walking-onnx`](https://huggingface.co/xiaofengzi/microduck-walking-onnx).
It is a 61→512→256→128→14 MLP (197,896 parameters including the baked
observation normalizer). The MJCF, meshes, and BAM settings are vendored
verbatim from the training repo (`mjlab-microduck`) so this demo stays
self-contained and the replay runs the same asset the policy was trained on.

## `velstand.onnx` — the second walking network

`models/microduck/velstand.onnx` is the other half of the `make walk-swap` demo:
Pollen's default *walking* policy since their v5, from the same published set
([`pollen-robotics/microduck-policies`](https://huggingface.co/pollen-robotics/microduck-policies),
Apache License 2.0). Their manifest describes it as a gait that also stands still
at zero command — which is why there is no separate standing network to swap in —
and records the training behind the file: task
`Mjlab-VelStand-Rough-Backlash-MicroDuck`, run
`pollen-robotics/mjlab_microduck/0045i2si` @ checkpoint 2749, in
`pollen-robotics/microduck_rl` @ `3750776` (branch `improve_standup_velstand`), the
export dated 2026-10-12.

It is deliberately *not* a drop-in twin of `BEST_alpha_walking.onnx`: same 61→14
contract, same `onnx_rl` controller, same `shinro_step` ABI, different weights and a
different baked normalizer — a genuinely different network driving the same duck.
That is what makes the swap worth running.

```
sha256  2fc7f8ee6acd7d0d3f66a99927b5d5a56589aeed02ec714de329b252d5ccf492
```

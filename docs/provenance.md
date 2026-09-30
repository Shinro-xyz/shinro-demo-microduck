*Part of [shinro-demo-microduck](../README.md).*

# Provenance

`BEST_alpha_walking.onnx` is the ONNX export of the deployed Microduck walking
policy: Weights & Biases run `441tzs6d` @ `model_3750`, published at
[`xiaofengzi/microduck-walking-onnx`](https://huggingface.co/xiaofengzi/microduck-walking-onnx).
It is a 61→512→256→128→14 MLP (197,896 parameters including the baked
observation normalizer). The MJCF, meshes, and BAM settings are vendored
verbatim from the training repo (`mjlab-microduck`) so this demo stays
self-contained and the replay runs the same asset the policy was trained on.

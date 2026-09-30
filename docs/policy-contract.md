*Part of [shinro-demo-microduck](../README.md).*

# The policy contract

Every Microduck policy shares this layout so a runtime can hot-swap ONNX files
without touching the glue. `src/shinro_demo_microduck/contract.py` is the
single source of truth, and `verify_against_onnx()` checks it against the ONNX
file's own exporter metadata (joint order, home pose, command block, action
scale) — a mismatch fails loudly instead of silently driving the robot with the
wrong joint order.

**Observation, 61 D:**

| slice | term | notes |
| ----- | ---- | ----- |
| `0:3` | `base_ang_vel` | IMU gyro, trunk frame |
| `3:6` | `projected_gravity` | gravity direction in the trunk frame |
| `6:20` | `joint_pos` | current − home pose, rad |
| `20:34` | `joint_vel` | rad/s |
| `34:48` | `actions` | the previous action |
| `48:51` | `twist` command | `[vx, vy, wz]` |
| `51:55` | `head_pose` command | `[neck_pitch, head_pitch, head_yaw, head_roll]` |
| `55:61` | `body_pose` command | `[x, y, z, roll, pitch, yaw]` |

**Action, 14 D:** `q_target = home_pose + action * action_scale`, with
`action_scale = 1.0` (read from the ONNX metadata, not assumed). Servo order is
`left_hip_yaw … left_ankle, neck_pitch … head_roll, right_hip_yaw … right_ankle`
— asserted against the model's actuator order at construction.

A command slot an env does not use is ZERO-PADDED, never removed. The walking
policy only rewards `twist`; `head_pose` is a small secondary objective and
`body_pose` was trained at weight 0 (kept alive so a later curriculum can use
it). The demo feeds zeros there and commands `twist`.

## Three things the layout pins down

- **The command block is zero-padded, never trimmed.** `twist` is what the walking
  policy rewards; `head_pose` is a small secondary objective and `body_pose` was
  trained at weight 0 (kept alive so a later curriculum can use it). Deleting an
  unused slot would move every byte after it and invalidate the whole policy family.
- **The action is an offset, not an angle.** `q_target = HOME + action`, with
  `action_scale = 1.0` read from the ONNX metadata rather than assumed — the policy
  outputs displacements from the home pose.
- **`last_action` is fed back** at `obs[34:48]`. That is why the compiled graph needs
  no recurrent ports: the policy's own previous output returns to it through the
  observation.

`COMMAND_BLOCK_SLICES` (the 13-D block) and `COMMAND_SLICES` (where each command
lands in the 61-D observation) are deliberately separate mappings — see the
[data flow in the README](../README.md#how-a-command-reaches-the-robot).

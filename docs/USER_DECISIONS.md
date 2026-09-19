# User decisions required

No final choice is made in this plan. The values below are recommendations
only and remain `PENDING_USER_DECISION` until the corresponding audit evidence
exists.

| ID | Options | Recommendation | Why | Blocking? |
|---|---|---|---|---|
| D1 NETWORK | NET-A SSH tunnel; NET-B Tailscale/private VPN; NET-C self-managed WireGuard | NET-A for P0/P1 bring-up; NET-B for later long-running experiments | A is the smallest encrypted bring-up path; B reduces repeated tunnel management | Yes for online inference; no for offline recorder |
| D2 ACTION | ACTION-A absolute joint position + gripper; ACTION-B joint delta + gripper; ACTION-C EE delta pose + gripper | ACTION-A | Closest to teach trajectories and avoids premature IK/frame conversion | Yes for training transform and execution |
| D3 GRIPPER TEACH INPUT | G-A keyboard; G-B foot pedal/external button; G-C official teach control | Decide after hardware audit; G-B is the operator-friendly fallback | Teach mode and gripper authority are firmware/hardware dependent | Yes for training-ready demonstrations |
| D4 FIRST TASK | TASK-A single-object pick-place; TASK-B color-conditioned pick; TASK-C multi-object pick-place | TASK-A | Smallest closed-loop milestone; B adds language grounding after baseline | Yes for the first real milestone |
| D5 INITIAL CHECKPOINT | CKPT-A `pi05_base`; CKPT-B `pi05_droid` | CKPT-A | New embodiment should not inherit DROID semantics by default | Yes for training/inference reproducibility |

## Default sequence, not a selection

```text
NET-A → NET-B (if long-running remote use is needed)
ACTION-A (after teach/controller/OpenPI audits)
G-A/G-B/G-C (after PiPER teach audit)
TASK-A → TASK-B → TASK-C
CKPT-A, with CKPT-B as a clearly labeled comparison if useful
```

The agent may fill recommendations and evidence requirements, but may not fill
the `selected` field for any D1–D5.

# GPU/data-server setup plan

This is a plan for the remote server. The current turn does not install OpenPI,
download weights, start a policy service, or change GPU workloads.

## Audit before setup

Record a fresh snapshot of:

- GPU model, count, UUIDs and VRAM;
- driver/CUDA/OS;
- storage and free space;
- network interfaces and route constraints;
- existing processes/workloads and ports.

The possible RTX PRO 6000 Blackwell 96GB is only a hypothesis until measured.
If GPU3 has an existing workload, it is `DO NOT TOUCH` by default. Any later
GPU process uses an explicit UUID/device selection, never an implicit
`--gpus all` equivalent.

## Planned server tree

```text
third_party/openpi/              pinned official source
server/policy_server/            serving wrapper, health, metadata
server/training/                 transforms, norm stats, train commands
server/dataset_tools/            validation and conversion
datasets/incoming/               verified transfer staging
datasets/raw/                    immutable canonical raw
datasets/lerobot/                derived LeRobot dataset
datasets/manifests/              source/derived mappings
checkpoints/                     server-only frozen checkpoints
reports/                         auditable results
logs/                            runtime logs
cache/                           HF/UV/OpenPI/cache roots
```

## Setup order after P0

1. Pin the official OpenPI checkout and submodules.
2. Configure project-local cache roots under `07_pi0.5/cache/`.
3. Create the minimal server environment and verify imports/entry points.
4. Run official dummy inference without a robot.
5. Load a real laptop-recorded observation only after P5, still without
   control.
6. Implement and test PiPER transforms and norm-stat provenance.
7. Train only after the phase gates allow it.
8. Freeze checkpoint/config/norm hashes before shadow inference.

The first server test is single-GPU and read-only with respect to the robot.
Multi-GPU is a later measured decision, not a setup default.

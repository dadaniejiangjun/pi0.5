# OpenPI provenance and contract

Status: `RESEARCH_SNAPSHOT / LOCAL_PIN_PENDING`

## Authority

- Repository: `https://github.com/Physical-Intelligence/openpi`
- Required branch: `main` unless a later experiment records an explicit tag or
  commit.
- Local checkout path: `third_party/openpi/`
- Required checkout method: `git clone --recurse-submodules`, followed by
  `git rev-parse HEAD` and `git submodule status`.
- Provenance record: `provenance/openpi_contract.json`

The official repository history observed during this planning pass showed the
`main` branch at commit
`215abfb217dbac7d5f1273282331b9b1866c0479`. This is a research snapshot, not
an authorization to use an unpinned moving branch. P0 must record the exact
local commit and submodule SHAs before any environment is installed or any
checkpoint is loaded.

## Submodules observed in the official repository

The official `.gitmodules` currently declares:

- `third_party/aloha` → `https://github.com/Physical-Intelligence/aloha.git`
- `third_party/libero` → `https://github.com/Lifelong-Robot-Learning/LIBERO.git`

Their exact local SHAs are intentionally unfilled until the repository is
checked out. Do not infer them from another project.

## π0.5 names and the important distinction

The current official configuration source contains, among others:

- `pi05_aloha` and `pi05_droid` inference configurations;
- `pi05_droid_finetune` for a smaller custom LeRobot-format dataset;
- `pi05_full_droid_finetune` for full DROID RLDS training;
- `pi05_base` as a base checkpoint/assets path in weight-loading examples.

Therefore this project treats the user-requested `pi05_base` and `pi05_droid`
as two different objects until P0 confirms the local source:

```text
pi05_base  = checkpoint/asset identity, not assumed to be a config name
pi05_droid = official DROID config, not assumed to be a PiPER contract
```

The official DROID transform currently combines six joint values with one
gripper value, maps two observed images into the model image slots, masks a
third slot for π0/π0.5, and returns a DROID-specific action slice. Those
semantics are evidence about DROID only. They are not evidence for PiPER
firmware, units, camera frames, action horizon, or controller behavior.

## Required P0 inspection

After local checkout, record hashes and the exact inspected lines/files for:

1. branch, commit, tag state, and submodules;
2. `src/openpi/training/config.py` configs and weight loaders;
3. `src/openpi/policies/droid_policy.py` and the closest custom-data example;
4. `src/openpi/transforms.py` padding, normalization, image resize and output
   behavior;
5. `scripts/serve_policy.py`, `docs/remote_inference.md`, and
   `packages/openpi-client`;
6. backend and dependency lock information;
7. policy metadata and checkpoint asset layout.

P0 produces the filled `provenance/openpi_contract.json`. No checkpoint is
downloaded in this planning turn.

## Contract adopted for planning, not yet frozen

The laptop will send model-sized `uint8` base and wrist RGB observations after
the official preprocessing path is audited. The current official remote
inference documentation uses client-side resize/pad to a typical 224×224 input
and a WebSocket policy server with default port 8000. This project adds a
private/encrypted transport requirement and a laptop-side safety boundary.

The custom PiPER path must define and test:

```text
PiperInputs
PiperOutputs
PiperDataConfig
PiperTrainConfig
```

The transformation is not accepted until a 7D PiPER state/action round trip,
normalization audit, padding/unpadding audit, and online/offline equivalence
test all pass.

## Official references

- OpenPI remote inference:
  https://github.com/Physical-Intelligence/openpi/blob/main/docs/remote_inference.md
- OpenPI serving entry point:
  https://github.com/Physical-Intelligence/openpi/blob/main/scripts/serve_policy.py
- OpenPI training configuration:
  https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/training/config.py
- OpenPI DROID transform:
  https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/policies/droid_policy.py
- OpenPI submodules:
  https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/.gitmodules

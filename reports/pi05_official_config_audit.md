# Official π0.5 configuration audit

Recorded against:

- repository: `Physical-Intelligence/openpi`
- branch: `main`
- commit: `215abfb217dbac7d5f1273282331b9b1866c0479`
- local source: `third_party/openpi/`
- Python: `3.11.16`
- lock: `third_party/openpi/uv.lock`

## Environment result

`OPENPI_ENV = PASS_WITH_WARNING`

The frozen lock synchronized successfully in the project `.venv` without the
RLDS group. Imports for `openpi`, `openpi_client`, `jax`, `torch`,
`transformers` and `lerobot` passed. JAX enumerated the selected GPU0 as
`cuda:0`.

The installed Torch 2.7.1 CUDA12 wheel warns that the detected Blackwell
`sm_120` device is outside the wheel's compiled architecture list. This is a
Torch-backend warning. The official base smoke below uses the official JAX
path; no Torch policy path is claimed by this report.

## Config identity

| Name | Current official meaning | Relevant fields |
|---|---|---|
| `pi05_base` | base checkpoint/assets path | used by `pi05_aloha`, `pi05_libero` and full-DROID weight loaders |
| `pi05_aloha` | inference config for π0.5 ALOHA-style input | `Pi0Config(pi05=True)`, default action dimension 32 and default horizon 50; ALOHA transform outputs its own 14D action slice |
| `pi05_droid` | inference config for π0.5 DROID input | `Pi0Config(pi05=True, action_horizon=15)`; DROID-specific state/image/action transform |
| `pi05_libero` | fine-tuning/inference config | `Pi0Config(pi05=True, action_horizon=10, discrete_state_input=False)` and `pi05_base` weight loader |
| `pi05_droid_finetune` | custom small DROID LeRobot fine-tune example | action dimension 32, horizon 16, DROID norm/stat assumptions |

Important: `pi05_base` is not a `TrainConfig` name in this commit. The first
load sanity uses `pi05_aloha + pi05_base` and is explicitly not a PiPER policy.

## Official image/state transform observations

- `ModelTransformFactory` resizes images to 224×224 and pads state/actions to
  the model action dimension.
- OpenPI's remote client example uses `uint8` images and
  `resize_with_pad` on the client.
- The ALOHA example expects a 14D ALOHA state and ALOHA camera names.
- The DROID example constructs its own state and image-slot/mask convention;
  it must not be copied as PiPER semantics.

The PiPER contract remains project-local: STATE_V1/ACTION_V1 are 7D
`[q1..q6, gripper]` with canonical radian/meter units, while raw SDK scales and
real ranges remain unverified. The custom scaffold does not connect to CAN or
feed a base-model output to a PiPER command path.

## Official server observation

`scripts/serve_policy.py` in this commit uses the official
`WebsocketPolicyServer` with default port 8000 but passes `host="0.0.0.0"`.
This is not acceptable for this project. `server/serve_policy_local.py`
instantiates the same official server class with a hard-coded
`host="127.0.0.1"` and rejects non-local bind requests.

## Source files audited

- `src/openpi/training/config.py`
- `src/openpi/models/pi0_config.py`
- `src/openpi/policies/aloha_policy.py`
- `src/openpi/policies/droid_policy.py`
- `src/openpi/transforms.py`
- `scripts/serve_policy.py`
- `src/openpi/serving/websocket_policy_server.py`
- `packages/openpi-client/src/openpi_client/image_tools.py`
- `packages/openpi-client/src/openpi_client/websocket_client_policy.py`

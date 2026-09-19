# 07_pi0.5 project rules

## GitHub push policy

The canonical project remote is:

```text
https://github.com/dadaniejiangjun/pi0.5.git
```

Push only the project root repository. `third_party/openpi` must remain the
official `Physical-Intelligence/openpi` submodule at the pinned commit recorded
in `provenance/openpi_git.json`; never replace it with a fork, mirror, copied
snapshot, or a modified upstream checkout.

Before every push:

1. Confirm `git remote -v` and the current branch.
2. Review `git status`, `git diff --cached`, and `git diff --check`.
3. Check the staged file list for credentials, CAN/hardware data, private keys,
   `.venv`, caches, checkpoints, raw datasets, and files larger than 100 MiB.
4. Preserve provenance files and project documentation; do not rewrite history
   or force-push unless the user explicitly authorizes it.
5. Push the reviewed commit to the named remote and verify the remote branch
   and commit ID afterward.

The following remain local-only unless the user explicitly creates a separate
release procedure: `.venv/`, `cache/`, `checkpoints/`, runtime logs, raw or
incoming datasets, machine secrets, and real-robot captures. No Git operation
may connect to CAN, enable/reset a PiPER, or start robot motion.

Credentials must be supplied by the user's configured Git credential helper,
SSH agent, or an interactive HTTPS prompt. Never place tokens or private keys
in this repository, commit message, remote URL, or project reports.
## Phase status authority

The unique machine-readable authority for phase status is
`docs/PI05_PIPER_PHASE_GATES.json`. After any phase completion, synchronize
all three locations:

- `docs/PI05_PIPER_PHASE_GATES.json`
- the current-status block in `README.md`
- `provenance/deployment_manifest.json`

Status reconciliation must use the declared evidence sources and must
not be used to imply a new experiment, inference, training run, server
benchmark, CAN access, real data collection, or autonomous execution.
